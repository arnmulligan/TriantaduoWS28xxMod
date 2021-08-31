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

#ifndef TDWS28XX_H
#define TDWS28XX_H

#if ! defined ARDUINO_TEENSY40 && ! defined ARDUINO_TEENSY41 && ! defined ARDUINO_TEENSY_MICROMOD
#error "Sorry, TriantaduoWS28xxMod is only compatible with Teensy 4.0, Teensy 4.1 and Teensy MicroMod."
#endif

#include <Arduino.h>
#include <DMAChannel.h>
class FlexIOHandler;

namespace TDWS28XX {

union Color {
  struct {
    uint8_t _;
    uint8_t blue;
    uint8_t green;
    uint8_t red;
  } RGB;
  struct {
    uint8_t _;
    uint8_t blue;
    uint8_t red;
    uint8_t green;
  } GRB;
  struct {
    uint8_t white;
    uint8_t blue;
    uint8_t red;
    uint8_t green;
  } GRBW;
  uint32_t raw;
} __attribute__((packed));

inline Color rgb(uint8_t r, uint8_t g, uint8_t b) { return { 0, b, g, r }; }
inline Color grb(uint8_t g, uint8_t r, uint8_t b) { return { 0, b, r, g }; }
inline Color grbw(uint8_t g, uint8_t r, uint8_t b, uint8_t w) { return { w, b, r, g }; }

enum FlexIOModule { FLEXIO1 = 0, FLEXIO2 = 1 }; // unfortunately flex 3 has no DMA support
enum ChannelType { RGB, GRB, GRBW };
enum ColorCapability { TRICOLOR, QUADCOLOR }; // adding white requires additional RAM
enum BufferMode {
  SINGLE_BUFFER_BLOCKING, // update pixels only upon flushBuffer(); see bufferReady()
  SINGLE_BUFFER, // update pixels continuously (may cause artifacts)
  DOUBLE_BUFFER // update pixels continuously but use double buffering
};

struct FlexPins {
  uint8_t SRCLK; // shift register shift clock
  uint8_t RCLK; // ... latch clock
  uint8_t SER; // ... shift data
};
// Pins available Teensy 4.0:
//   FLEXIO1: 2, 3, 4, 5, 33
//   FLEXIO2: 6, 7, 8, 9, 10, 11, 12, 13, 32
// Pins available Teensy 4.1:
//   FlexIO1: 2, 3, 4, 5, 33, 49, 50, 52, 54
//   FLEXIO2: 6, 7, 8, 9, 10, 11, 12, 13, 32, 34, 35, 36, 37
// Pins available Teensy MicroMod:
//   FlexIO1: 2, 3, 4, 5, 33
//   FLEXIO2: 6, 7, 8, 9, 10, 11, 12, 13, 32, 40, 41, 42, 43, 44, 45

struct InternalProperties // internal use only
{
  uint16_t leds;
  ColorCapability cc;
  BufferMode bm;
  size_t bsz;
  uint8_t *bptr;
};

template<uint16_t maximumLedsPerStrip,
  ColorCapability colorCapability = QUADCOLOR,
  BufferMode bufferMode = SINGLE_BUFFER_BLOCKING>
struct PixelBuffer
{
  operator const InternalProperties*() const { return &p; }
  uint8_t buffer[sizeof(uint32_t) * maximumLedsPerStrip
        * ((colorCapability == QUADCOLOR) ? 32 : 24)
        * ((bufferMode == DOUBLE_BUFFER) ? 2 : 1)];
  const InternalProperties p = {
    maximumLedsPerStrip,
    colorCapability,
    bufferMode,
    sizeof(buffer) / ((bufferMode == DOUBLE_BUFFER) ? 2 : 1),
    buffer
  };
};

class PixelDriver
{
  public:
    FLASHMEM PixelDriver(const InternalProperties* ip_);
    FLASHMEM virtual ~PixelDriver();
    FLASHMEM void setChannelType(uint8_t channel, ChannelType type); // channels 0 -> 31
    FLASHMEM bool begin(FlexIOModule flexIOModule = FLEXIO1, FlexPins flexPins = { 2, 3, 4 }); // returns true on success
    
    void flipBuffers(void); // for double buffer mode
    void flushBuffer(void) { flipBuffers(); } // for single buffer modes
    bool bufferReady(); // returns true if flush has completed and the pixel buffer can safely be modified
    
    void setLed(uint8_t channel, uint16_t ledIndex, const Color &color) {
      setActiveLed(channel, ledIndex, color);
    }
    void setActiveLed(uint8_t channel, uint16_t ledIndex, const Color &color) {
      setLed(channel, ledIndex, color, activeBuffer);
    }
    void setInactiveLed(uint8_t channel, uint16_t ledIndex, const Color &color) {
      setLed(channel, ledIndex, color, inactiveBuffer);
    }
    
    Color getLed(uint8_t channel, uint16_t ledIndex) {
      return getActiveLed(channel, ledIndex);
    }
    Color getActiveLed(uint8_t channel, uint16_t ledIndex) {
      return getLed(channel, ledIndex, activeBuffer);
    }
    Color getInactiveLed(uint8_t channel, uint16_t ledIndex) {
      return getLed(channel, ledIndex, inactiveBuffer);
    }

  private:
    void dmaIsr(void);
    void configurePins(bool enable);
    void configureFlexIO(bool enable);
    void configurePll5(bool enable);
    void configureDma(bool enable);
    void setLed(uint8_t channel, uint16_t ledIndex, const Color &color, volatile uint32_t *buffer);
    Color getLed(uint8_t channel, uint16_t ledIndex, volatile uint32_t *buffer);

    static unsigned instanceCount;
    static PixelDriver *instances[2];
    
    const InternalProperties * const ip;
    const unsigned dmasDataSegmentsCount;
    FlexIOModule flexIOModule;
    FlexPins flexPins;
    FlexIOHandler *pFlex;
    DMAChannel dmaChannel;
    DMASetting dmasPresetZeros;
    DMASetting dmasSetOnes;
    DMASetting dmasDataSegments[4];
    DMASetting dmasSetZeros;
    DMASetting dmasLoopZeros;
    ChannelType channelTypes[32];
    volatile uint32_t * activeBuffer;
    volatile uint32_t * inactiveBuffer;
    
    friend void dmaIsr0();
    friend void dmaIsr1();
};

} // namespace TDWS28XX

#endif // TDWS28XX_H
