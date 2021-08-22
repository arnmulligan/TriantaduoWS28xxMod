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

/*
   Move a colored stripe back and forth along the pixel array to demonstrate
   multi-channel capability.
*/

#include <TDWS28XX.h>
using namespace TDWS28XX;

const uint8_t NumberOfChannels = 32; // number of shift register outputs in use (max 32)
const uint16_t NumberOfLedsPerChannel = 100; // total LEDs connected to a shift register output

const Color BaseColor = grb(0, 0, 10);
const Color ForeColor = grb(64, 64, 64);


// PixelBuffer<NumberOfLedsPerChannel, TRICOLOR, DOUBLE_BUFFER> pb; // store large buffer in RAM1
DMAMEM PixelBuffer<NumberOfLedsPerChannel, TRICOLOR, DOUBLE_BUFFER> pb; // store large buffer in RAM2

PixelDriver pd(pb);
uint16_t curPosition = 0; // current position of colored stripe
bool forward = true; // direction of movement of the stripe

void paintBackground(const Color &color) {
  for (uint8_t i = 0; i < NumberOfChannels; ++i) {
    for (uint16_t j = 0; j < NumberOfLedsPerChannel; ++j) {
      pd.setInactiveLed(i, j, color);
    }
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(333);

  if (! pd.begin()) {
    Serial.println("configuration error");
    for (;;);
  }

  // some configuration
  for (uint8_t i = 0; i < NumberOfChannels; ++i) {
    pd.setChannelType(i, GRB);
  }

  // initialise the inactive buffer with background color
  paintBackground(BaseColor);

  // set each first pixel to a different color to draw a stripe and to
  // make each LED strip visually unique
  for (uint16_t i = 0; i < NumberOfChannels; ++i) {
    Color color = grb(
      (uint8_t)random(ForeColor.GRB.green + 1),
      (uint8_t)random(ForeColor.GRB.red + 1),
      (uint8_t)random(ForeColor.GRB.blue + 1));
    pd.setInactiveLed(i, 0, color);
  }

  // swap active buffer for inactive buffer thus updating the display
  pd.flipBuffers();

  // initialise the formerly active buffer
  paintBackground(BaseColor);
}

void loop() {
  delay(50);

  const uint16_t newPosition = forward ? curPosition + 1 : curPosition - 1;

  // draw stripe in new position
  for (uint16_t i = 0; i < NumberOfChannels; ++i) {
    pd.setInactiveLed(i, newPosition, pd.getActiveLed(i, curPosition));
  }

  // display the new stripe
  pd.flipBuffers();

  // erase old stripe
  for (uint16_t i = 0; i < NumberOfChannels; ++i) {
    pd.setInactiveLed(i, curPosition, BaseColor);
  }

  // housekeeping
  curPosition = newPosition;
  if (curPosition == 0 || curPosition + 1 == NumberOfLedsPerChannel) {
    forward = !forward;
  }
}
