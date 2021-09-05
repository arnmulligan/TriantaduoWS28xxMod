/* MIT License

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

/*
   Display a series of 180x16 pixel images from a MicroSD card on a Teensy 4.1.

   Only 8 channels are used in this example, which isn't an efficient use of the library as
   the same amount of RAM is used whether 1 or 32 channels are used. Better to use all 32
   channels to avoid waste. The LED pixel strips are connected in pairs, one strip forward,
   one strip back. The first strip of each pair is connected to a shift register output pin;
   the second strip is connected to the end of the first strip and so is running in the
   opposite direction to the first. Top left of the PNG image is the first shift register
   channel.
   
   The PNG files must be encoded in 8 bit per colour channel RGB and be of the correct
   dimensions for the display. See "GIMP_export_PNG_settings.png" for a screenshot of
   suitable settings in GIMP. The PNG files must reside in the root directory of the
   MicroSD card.
*/

#include <TDWS28XX.h>
#include <SdFat.h> // comes with Teensy installation
#include <PNGdec.h> // PNGdec by Larry Bank, install with library manager
using namespace TDWS28XX;


const uint8_t NumberOfChannels = 8; // number of shift register outputs in use (max 32)
const uint16_t NumberOfPixelsPerChannel = 360; // total LEDs connected to a shift register output
const uint16_t NumberOfPixelsPerRow = 180; // pixels per display row (must divide into NumberOfPixelsPerChannel)
const unsigned long SlideShowIntervalMs = 3000; // delay between each displayed image
const uint8_t BrightnessPercent = 20; // dim the display to the corresponding percent


const uint16_t NumberOfRows = NumberOfPixelsPerChannel / NumberOfPixelsPerRow * NumberOfChannels;
DMAMEM PixelBuffer<NumberOfPixelsPerChannel, TRICOLOR, SINGLE_BUFFER_BLOCKING> pb;
PixelDriver pd(pb);
PNG png;
SdFs sdfs;
FsFile directory;
unsigned long lastSlideStartMs;
bool loadingBuffer;
uint16_t rowsBuffered;
extern const uint8_t gamma8[];

struct position {
  uint16_t x;
  uint16_t y;
};

// translate logical pixel position in the PNG to physical display position
inline position convert(const position &logical) {
  position physical;
  physical.y = logical.y / (NumberOfPixelsPerChannel / NumberOfPixelsPerRow);
  uint16_t a = logical.y % (NumberOfPixelsPerChannel / NumberOfPixelsPerRow);
  physical.x = (a & 1)
    ? a * NumberOfPixelsPerRow + (NumberOfPixelsPerRow - 1 - logical.x)
    : a * NumberOfPixelsPerRow + logical.x;
  return physical;
}

void* pngOpen(const char *filename, int32_t *size) {
  static FsFile f;
  Serial.printf("opening %s\n", filename);
  f = sdfs.open(filename);
  *size = f.size();
  return &f;
}

void pngClose(void *file) {
  FsFile *f = static_cast<FsFile *>(file);
  if (f) f->close();
}

int32_t pngRead(PNGFILE *handle, uint8_t *buffer, int32_t length) {
  FsFile *f = static_cast<FsFile *>(handle->fHandle);
  if (! f || !*f) return 0;
  return f->read(buffer, length);
}

int32_t pngSeek(PNGFILE *handle, int32_t position) {
  FsFile *f = static_cast<FsFile *>(handle->fHandle);
  if (! f || !*f) return 0;
  return f->seek(position);
}

inline uint8_t dim(uint8_t v) {
  return v * BrightnessPercent / 100;
}

void pngDraw(PNGDRAW *pDraw) {
  uint8_t* b = pDraw->pPixels;
  uint16_t y = static_cast<uint16_t>(pDraw->y);
  Color c;
  for (uint16_t x = 0; x < NumberOfPixelsPerRow; ++x) {
    position p = convert(position{x, y});
    c.GRB.red = dim(gamma8[*b++]);
    c.GRB.green = dim(gamma8[*b++]);
    c.GRB.blue = dim(gamma8[*b++]);
    pd.setPixel(p.y, p.x, c);
  }
}

void setup() {
  Serial.begin(115200);

  if (! pd.begin()) {
    Serial.println("configuration error");
    for (;;);
  }

  for (uint8_t i = 0; i < NumberOfChannels; ++i) {
    pd.setChannelType(i, GRB);
  }
  
  if (! sdfs.begin(SdioConfig(FIFO_SDIO)) || ! (directory = sdfs.open("/"))) {
    Serial.println("unable to access SD card");
    for (;;);
  }
}

void loop() {
  if (loadingBuffer) {
    // strangely there doesn't seem to be any way to determine when a PNG has
    // been fully decoded, so the solution here is to just count the rows
    int rc = png.decode(nullptr, 0);
    if (rc != PNG_SUCCESS) {
      png.close();
      Serial.println("decoding PNG error");
      loadingBuffer = false;
    } else if (++rowsBuffered >= NumberOfRows) {
      png.close();
      loadingBuffer = false;
      pd.flushBuffer();
    }
  } else if (millis() - lastSlideStartMs > SlideShowIntervalMs && pd.bufferReady()) {
    FsFile f = directory.openNextFile();
    if (! f) {
      directory.rewindDirectory();
      goto SKIP_THIS_FILE;
    }
    if (f.isDirectory()) goto SKIP_THIS_FILE;
    char fn[16];
    size_t l = f.getName(fn, sizeof(fn));
    if (l < 5) goto SKIP_THIS_FILE;
    if (strcasecmp(fn + l - 4, ".png")) goto SKIP_THIS_FILE;
    int rc = png.open(fn, pngOpen, pngClose, pngRead, pngSeek, pngDraw);
    if (rc != PNG_SUCCESS) goto SKIP_THIS_FILE;
    if (png.getWidth() != NumberOfPixelsPerRow
        || png.getHeight() != NumberOfRows
        || png.getPixelType() != PNG_PIXEL_TRUECOLOR) {
      png.close();
      Serial.print("unsupported file format: ");
      Serial.println(fn);
      goto SKIP_THIS_FILE;
    }
    
    // looks like a valid PNG
    loadingBuffer = true;
    rowsBuffered = 0;
    lastSlideStartMs = millis();
  }
  SKIP_THIS_FILE: ;

  /* do other useful stuff here */
}

// gamma8 from: https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix
//
const uint8_t gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255
};
