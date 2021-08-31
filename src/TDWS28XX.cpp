/* MIT License

Original concept:
Copyright (c) 2020 Ward Ramsdell

Extensively revised by:
Copyright (c) 2021 Arn Mulligan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "TDWS28XX.h"
#include <FlexIO_t4.h>

static const uint32_t Zeros = 0u;
static const uint32_t Ones = ~0u;

/* Some pixels require almost 300uS reset interval, so use that as default. */
/* 1.25uS (bit duration) * 240 = 300uS (reset interval) */
static unsigned const BitTimesPerResetTime = 240;

/* Maximum possible value of BITER/CITER field. */
static uint16_t const MaxDMAIterationsPerTCD = DMA_TCD_BITER_MASK;

static bool dmaEnabled(const DMAChannel &c) {
  return DMA_ERQ & (1 << c.channel);
}

namespace TDWS28XX {

unsigned PixelDriver::instanceCount = 0;
PixelDriver *PixelDriver::instances[] = { };

void dmaIsr0() {
  PixelDriver::instances[0]->dmaIsr();
}

void dmaIsr1() {
  PixelDriver::instances[1]->dmaIsr();
}

void (*dmaISRs[])() = { dmaIsr0, dmaIsr1 };

PixelDriver::PixelDriver(const InternalProperties* ip_)
  : ip(ip_)
  , dmasDataSegmentsCount(ip->bsz / sizeof(uint32_t) / MaxDMAIterationsPerTCD + 1)
{
  const size_t c = sizeof(dmasDataSegments) / sizeof(*dmasDataSegments);
  if (dmasDataSegmentsCount > c) *const_cast<unsigned*>(&dmasDataSegmentsCount) = c;
}

PixelDriver::~PixelDriver() {
  --instanceCount;
  
  /* Disable peripherals */
  configureDma(false);
  configureFlexIO(false);
  configurePins(false);
  if (! instanceCount) configurePll5(false);
  
  instances[flexIOModule] = nullptr;
}

bool PixelDriver::begin(FlexIOModule flexIOModule_, FlexPins flexPins_) {
  flexIOModule = flexIOModule_;
  flexPins = flexPins_;
  
  /* Sanity */
  if (! ip->leds) return false;
  if (flexIOModule > FLEXIO2) return false;
  if (pFlex) return false;

  /* Check this flex module isn't in use already */
  if (instances[flexIOModule]) return false;
  
  /* Exlusive use of PLL5 is desirable */
  if (! instanceCount && ! (CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_POWERDOWN)) return false;
  
  /* Get a FlexIO channel */
  FlexIOHandler *pf = FlexIOHandler::flexIOHandler_list[flexIOModule];
  
  /* Validate supplied pins */
  if (pf->mapIOPinToFlexPin(flexPins.SRCLK) == 0xff) return false;
  if (pf->mapIOPinToFlexPin(flexPins.RCLK) == 0xff) return false;
  if (pf->mapIOPinToFlexPin(flexPins.SER) == 0xff) return false;
  
  /* Init instances for ISR use */
  instances[flexIOModule] = this;
  
  /* Set up buffer pointers */
  activeBuffer = reinterpret_cast<uint32_t*>(ip->bptr);
  inactiveBuffer = (ip->bm == DOUBLE_BUFFER)
    ? reinterpret_cast<uint32_t*>(ip->bptr + ip->bsz)
    : activeBuffer;
  
  /* Initialise buffers */
  memset(const_cast<uint32_t*>(activeBuffer), 0, ip->bsz);
  arm_dcache_flush((uint8_t *)activeBuffer, ip->bsz);
  memset(const_cast<uint32_t*>(inactiveBuffer), 0, ip->bsz);
  arm_dcache_flush((uint8_t *)inactiveBuffer, ip->bsz);

  /* Now configure the peripherals */
  pFlex = pf;
  if (! instanceCount) configurePll5(true);
  configurePins(true);
  configureFlexIO(true);
  configureDma(true);
  
  ++instanceCount;
  return true;
}

void PixelDriver::dmaIsr(void) {
  /* Disable interrupt in the TCD */
  dmasSetZeros.TCD->CSR &= ~DMA_TCD_CSR_INTMAJOR;

  /* Swap the buffer pointers in the TCDs */
  volatile uint32_t *bptr = activeBuffer;
  for (unsigned i = 0; i < dmasDataSegmentsCount; ++i) {
    dmasDataSegments[i].TCD->SADDR = bptr;
    bptr += MaxDMAIterationsPerTCD;
  }

  /* Clear the interrupt so we don't get triggered again */
  dmaChannel.clearInterrupt();
  __asm__ volatile ("DSB");
  __asm__ volatile ("ISB");
}

void PixelDriver::configurePins(bool enable) {
  uint8_t pm = enable ? OUTPUT : INPUT;
  uint32_t pc = enable
    ? (IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3))
    : (IOMUXC_PAD_PKE | IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_DSE(6));

  /* Basic pin setup */
  pinMode(flexPins.SRCLK, pm);
  pinMode(flexPins.RCLK, pm);
  pinMode(flexPins.SER, pm);

  /* High speed and drive strength configuration */
  *portControlRegister(flexPins.SRCLK) = pc;
  *portControlRegister(flexPins.RCLK) = pc;
  *portControlRegister(flexPins.SER) = pc;
}

void PixelDriver::configureFlexIO(bool enable) {
  IMXRT_FLEXIO_t *p = &pFlex->port();

  /* Start by resetting the peripheral */
  p->CTRL = FLEXIO_CTRL_SWRST;
  p->CTRL = 0;

  if (! enable) return;

  /* Configure flex clock: clock=PLL5, pred=/5, podf=/1 */
  /* Note: The pred of 4 (/5) isn't mentioned as a possible value of the FLEXIO#_CLK_PRED */
  /* 3 bit bitfield in the RT1060 Processor Reference Manual rev1 or rev2. However, in the */
  /* rev2 manual, similar 3 bit bitfields do show the value and indicate the corresponding */
  /* divider. It works, so no worries. */
  /* So the input clock of 768MHz is divided by 5 to get 153.6MHz */
  pFlex->setClockSettings(2, 4, 0);
  
  /* Set up the pin mux */
  pFlex->setIOPinToFlexMode(flexPins.SRCLK);
  pFlex->setIOPinToFlexMode(flexPins.RCLK);
  pFlex->setIOPinToFlexMode(flexPins.SER);

  /* Shifter configuration */
  p->SHIFTCTL[0] = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_TIMPOL
    | FLEXIO_SHIFTCTL_PINCFG(3)
    | FLEXIO_SHIFTCTL_PINSEL(pFlex->mapIOPinToFlexPin(flexPins.SER))
    | FLEXIO_SHIFTCTL_SMOD(2);
  p->SHIFTCTL[1] = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_TIMPOL
    | FLEXIO_SHIFTCTL_SMOD(2);
  p->SHIFTCTL[2] = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_TIMPOL
    | FLEXIO_SHIFTCTL_SMOD(2);

  p->SHIFTCFG[0] = FLEXIO_SHIFTCFG_INSRC;
  p->SHIFTCFG[1] = FLEXIO_SHIFTCFG_INSRC;
  p->SHIFTCFG[2] = FLEXIO_SHIFTCFG_INSRC;

  /* Timer configuration */
  p->TIMCFG[0] = FLEXIO_TIMCFG_TIMENA(2);
  p->TIMCFG[1] = FLEXIO_TIMCFG_TIMENA(2);
  
  p->TIMCTL[0] = FLEXIO_TIMCTL_TRGSEL(1) | FLEXIO_TIMCTL_TRGPOL
    | FLEXIO_TIMCTL_TRGSRC | FLEXIO_TIMCTL_PINCFG(3)
    | FLEXIO_TIMCTL_PINSEL(pFlex->mapIOPinToFlexPin(flexPins.SRCLK))
    | FLEXIO_TIMCTL_TIMOD(1);
  p->TIMCTL[1] = FLEXIO_TIMCTL_TRGSEL(1) | FLEXIO_TIMCTL_TRGPOL
    | FLEXIO_TIMCTL_TRGSRC | FLEXIO_TIMCTL_PINCFG(3)
    | FLEXIO_TIMCTL_PINSEL(pFlex->mapIOPinToFlexPin(flexPins.RCLK))
    | FLEXIO_TIMCTL_TIMOD(3);

  /* Using 8 bit baud counter mode so the least significant byte forms the */
  /* baud rate divider, in this case 2. So 153.6MHz / 2 = 76.8MHz, which is */
  /* the required output frequency for SRCLK. */
  p->TIMCMP[0] = 0x0000BF00;

  /* Using 16 bit counter mode so the whole value forms the baud rate */
  /* divider, in this case 64. So 153.6MHz / 64 = 2.4MHz, which is the */
  /* required frequency for RCLK. */
  p->TIMCMP[1] = 0x0000001F;

  /* Set up the values to be loaded into the shift registers at the beginning of each bit */
  p->SHIFTBUF[0] = Ones;
  p->SHIFTBUFBIS[1] = 0xAAAAAAAA;  // Debugging pattern should DMA fail to write SHIFTBUFBIS[1]
  p->SHIFTBUF[2] = Zeros;

  /* Enable DMA trigger on shifter 1 */
  p->SHIFTSDEN |= 0x00000002;
  
  /* Enable the FlexIO */
  p->CTRL = FLEXIO_CTRL_FLEXEN;
}

void PixelDriver::configurePll5(bool enable) {
  /* PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM) */
  /* where Fref = 24MHz */
  /* So in this instance we have PLLout = 24 * (32 + 0/1) = 768 MHz */
  
  /* Bypass PLL first */
  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC(3);
  CCM_ANALOG_PLL_VIDEO |= CCM_ANALOG_PLL_VIDEO_BYPASS | CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC(0);

  /* Shutdown PLL before changing parameters */
  CCM_ANALOG_PLL_VIDEO |= CCM_ANALOG_PLL_VIDEO_POWERDOWN;

  if (! enable) return;

  /* DIV: 32 */
  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_DIV_SELECT(127);
  CCM_ANALOG_PLL_VIDEO |= CCM_ANALOG_PLL_VIDEO_DIV_SELECT(32);

  /* NUM/DENOM: 0 */
  CCM_ANALOG_PLL_VIDEO_NUM = 0;
  CCM_ANALOG_PLL_VIDEO_DENOM = 1;
  
  /* Post divider: divide by 1 */
  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(3);
  CCM_ANALOG_PLL_VIDEO |= CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(2);
  
  /* Video divider: divide by 1 */
  CCM_ANALOG_MISC2 &= ~CCM_ANALOG_MISC2_VIDEO_DIV(3);
  CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_VIDEO_DIV(0);
  
  /* Power up */
  CCM_ANALOG_PLL_VIDEO |= CCM_ANALOG_PLL_VIDEO_ENABLE;
  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_POWERDOWN;

  /* Wait for the PLL to lock. If it doesn't lock, well, wait some more. */
  while (! (CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_LOCK));
  
  /* Disable bypass once it's locked. */
  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_BYPASS;
}

void PixelDriver::configureDma(bool enable) {
  IMXRT_FLEXIO_t *p = &pFlex->port();
  const FlexIOHandler::FLEXIO_Hardware_t *hw = &pFlex->hardware();
  dmaChannel.disable();

  if (! enable) {
    dmaChannel.detachInterrupt();
    return;
  }

  /* Set shift buffer 1 to zero first. Not sure why this is needed, but without */
  /* it, particularly for SINGLE_BUFFER_BLOCKING mode, sporadically an extra */
  /* pixel bit precedes the correct buffer and messes up the output. It would */
  /* be nice to understand why...? */
  dmasPresetZeros.sourceBuffer(&Zeros,4);
  dmasPresetZeros.destination(p->SHIFTBUFBIS[1]);
  dmasPresetZeros.replaceSettingsOnCompletion(dmasSetOnes);

  /* This TCD signifies the end of the pixel reset period: it sets shifter 0 */
  /* to ones and thus enables the high part of each following pixel bit. */
  dmasSetOnes.sourceBuffer(&Ones,4);
  dmasSetOnes.destination(p->SHIFTBUF[0]);
  dmasSetOnes.replaceSettingsOnCompletion(dmasDataSegments[0]);

  /* These TCDs are responsible for the bulk of the data transfer: they steer */
  /* pixel data from the frame buffer to shifter 1. Since there is a limit */
  /* imposed on the number of transfers per TCD, these TCDs are chained to */
  /* allow for maximum pixel strip length. */
  volatile uint32_t *bptr = activeBuffer;
  size_t remaining = ip->bsz / sizeof(uint32_t);
  for (unsigned i = 0; i < dmasDataSegmentsCount; ++i) {
    size_t sz = (remaining >= MaxDMAIterationsPerTCD) ? MaxDMAIterationsPerTCD : remaining;
    dmasDataSegments[i].sourceBuffer(bptr, sz * sizeof(uint32_t));
    dmasDataSegments[i].destination(p->SHIFTBUFBIS[1]);
    dmasDataSegments[i].replaceSettingsOnCompletion(dmasDataSegments[i+1]);
    bptr += MaxDMAIterationsPerTCD;
    remaining -= sz;
  }
  dmasDataSegments[dmasDataSegmentsCount-1].replaceSettingsOnCompletion(dmasSetZeros);

  /* This TCD is a precursor to the pixel reset period: it sets shifter 0 */
  /* to zero and thus disables the high part of each pixel bit that follows. */
  dmasSetZeros.sourceBuffer(&Zeros,4);
  dmasSetZeros.destination(p->SHIFTBUF[0]);
  dmasSetZeros.replaceSettingsOnCompletion(dmasLoopZeros);

  /* This TCD is responsible for the pixel reset delay: it sends a buffer */
  /* of zeros in a loop to shifter 1 (instead of pixel data) and requires */
  /* a manual configuration of the TCD. */
  DMABaseClass::TCD_t *tcd = dmasLoopZeros.TCD;
  tcd->SADDR = &Zeros;
  tcd->SOFF = 0;
  tcd->ATTR_SRC = 2;
  tcd->NBYTES = 4;
  tcd->SLAST = -4;
  tcd->BITER = BitTimesPerResetTime;
  tcd->CITER = BitTimesPerResetTime;
  dmasLoopZeros.destination(p->SHIFTBUF[1]);

  /* Configure FlexIO module to trigger DMA. */
  dmaChannel = dmasPresetZeros;
  dmaChannel.triggerAtHardwareEvent(hw->shifters_dma_channel[1]);
  
  if (ip->bm == SINGLE_BUFFER_BLOCKING) {
    dmasLoopZeros.disableOnCompletion();
  } else {
    /* Continuously refreshing the pixels so loop the TCDs. */
    dmasLoopZeros.replaceSettingsOnCompletion(dmasPresetZeros);
    if (ip->bm == DOUBLE_BUFFER) {
      /* Interrupt for pixel buffer switching. */
      dmaChannel.attachInterrupt(dmaISRs[flexIOModule]);
    }
    dmaChannel.enable();
  }
}

void PixelDriver::flipBuffers(void) {
  if (! pFlex) return;
  
  if (ip->bm == SINGLE_BUFFER_BLOCKING) {
    /* Update pixels when not being DMAed. */
    while (dmaEnabled(dmaChannel));
    dmaChannel = dmasPresetZeros;
    arm_dcache_flush((uint8_t*)activeBuffer, ip->bsz);
    dmaChannel.enable();
  }
  else if (ip->bm == DOUBLE_BUFFER) {
    /* Update pixels by swapping active and inactive frame buffers. */
    volatile uint32_t *t = activeBuffer;
    activeBuffer = inactiveBuffer;
    inactiveBuffer = t;
    arm_dcache_flush((uint8_t*)activeBuffer, ip->bsz); /* implicit dsb isb */

    /* This sets the ISR on completion flag of the TCD and the ISR handles the rest. */
    /* This is to synchronize the buffer swap with the frame blanking period in order to prevent tearing. */
    dmasSetZeros.TCD->CSR |= DMA_TCD_CSR_INTMAJOR;
  }
  else if (ip->bm == SINGLE_BUFFER) {
    /* Update pixels with risk of artifacts. */
    arm_dcache_flush((uint8_t*)activeBuffer, ip->bsz);
  }
}

bool PixelDriver::bufferReady() {
  return ! (ip->bm == SINGLE_BUFFER_BLOCKING && dmaEnabled(dmaChannel));
}

void PixelDriver::setLed(uint8_t channel, uint16_t ledIndex, const Color &color, volatile uint32_t *buffer) {
  if (channel > 31 || ledIndex >= ip->leds) return;

  const uint32_t channelMask = 1 << channel;
  uint32_t ledMask, ledValue;
  
  if (channelTypes[channel] == GRBW) {
    buffer += 32u * ledIndex;
    ledMask = 1 << 31;
    ledValue = color.raw;
  } else {
    buffer += 24u * ledIndex;
    ledMask = 1 << 23;
    ledValue = color.raw >> 8;
  }
  
  while (ledMask) {
    if (ledValue & ledMask) {
      *buffer |= channelMask;
    } else {
      *buffer &= ~channelMask;
    }
    ++buffer;
    ledMask >>= 1;
  }
}

Color PixelDriver::getLed(uint8_t channel, uint16_t ledIndex, volatile uint32_t *buffer) {
  if (channel > 31 || ledIndex >= ip->leds) return Color();
  
  const uint32_t channelMask = 1 << channel;
  unsigned count;
  Color c;
  
  if (channelTypes[channel] == GRBW) {
    buffer += 32u * ledIndex + 32u;
    count = 32;
  } else {
    buffer += 24u * ledIndex + 24u;
    count = 24;
  }
  
  while (count--) {
    --buffer;
    c.raw >>= 1;
    if (*buffer & channelMask) {
      c.raw |= 1 << 31;
    }
  }
  
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
  return c;
#pragma GCC diagnostic pop
}

void PixelDriver::setChannelType(uint8_t channel, ChannelType type) {
  /* Allows the user to change each channel to RGB, GRB, or GRBW formatting */
  if (channel >= 32) return;
  if (type == GRBW && ip->cc != QUADCOLOR) return;
  channelTypes[channel] = type;
}

} // namespace TDWS28XX
