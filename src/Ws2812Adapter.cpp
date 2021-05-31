//
// Created by Ben Hencke on 11/28/17.
//

/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Ws2812Adapter.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <Arduino.h>

#ifdef ESP8266
extern "C"
{
#include "eagle_soc.h"
#include "uart_register.h"
}
#endif

#ifdef ESP32
#include "esp32-hal-uart.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"

#endif

#ifdef ESP8266
static const uint8_t bits[4] = {
        0b11101111,
        0b11001111,
        0b11101110,
        0b11001110,
};


static inline void write(uint8_t c) {
#ifdef ESP32
    UART1.fifo.rw_byte = c;
#endif
#ifdef ESP8266
    Serial1.write(c);
#endif
}
#endif

void Ws2812Adapter::show(uint16_t numPixels, Ws2812PixelFunction cb) {
    int curPixel;
    uint8_t rgb[elements], buf[elements];

    memset(rgb, 0, elements);

    if (numPixels > 2500)
        numPixels = 2500;

    size_t bufferSize = numPixels * elements;
    setBuffer(bufferSize);
    //render and buffer pixels
    for (curPixel = 0; curPixel < numPixels; curPixel++) {
        cb(curPixel, rgb);
        int pixelOffset = curPixel * elements;
        //swap around rgb values based on mapping
        buffer[pixelOffset + rOffset] = rgb[0];
        buffer[pixelOffset + gOffset] = rgb[1];
        buffer[pixelOffset + bOffset] = rgb[2];
        if (elements == 4)
            buffer[pixelOffset + 3] = rgb[3];
    }

    //wait for any previous latch
    while (micros() - timer < 300) //use ws2813 timing
        yield();

#ifdef ESP8266
    //we need 12-16 bytes (elements * 4) free in the uart tx fifo for a whole pixel
    register int uartHighWatermark = 0x7f - (elements << 2);

    //stream out a pixel at a time to the uart fifo
    for (curPixel = 0; curPixel < numPixels; curPixel++) {
        int pixelOffset = curPixel * elements;
        buf[0] = buffer[pixelOffset];
        buf[1] = buffer[pixelOffset + 1];
        buf[2] = buffer[pixelOffset + 2];
        if (elements == 4)
            buf[3] = buffer[pixelOffset + 3];
        //wait for 12-16 bytes (elements * 4) free in the uart tx fifo before locking interrupts
        while((USS(UART1) >> USTXC) >= uartHighWatermark) {
            //busy loop, or should we yield?
        }
        os_intr_lock();

        for (uint8_t i = 0; i < elements; i++) {
            uint8_t c = buf[i];
            write(bits[c >> 6]);
            write(bits[(c >> 4) & 0x03]);
            write(bits[(c >> 2) & 0x03]);
            write(bits[c & 0x03]);
        }

        os_intr_unlock();

    }

    //wait for the last bits to send before starting latch timer
    Serial1.flush();

#endif

#ifdef ESP32

    //TODO when showPixels is async, wait for last showPixels to finish, + 300 micros
    uint8_t * pData = mRMTController.getPixelData(bufferSize);
    std::memcpy(pData, buffer.get(), bufferSize);

    //TODO rewrite showPixels to be async, not block, and record time of completion
    mRMTController.showPixels();
#endif

    timer = micros();
}

void Ws2812Adapter::setColorOrder(uint8_t o) {
    rOffset = (uint8_t) ((o & 3));
    gOffset = (uint8_t) (((o >> 2) & 3));
    bOffset = (uint8_t) (((o >> 4) & 3));
}

void Ws2812Adapter::setColorOrder(uint8_t o, bool hasWhite) {
    setColorOrder(o);
    elements = hasWhite ? 4 : 3;
}


void Ws2812Adapter::end() {
//    Serial1.end();
}

Ws2812Adapter::~Ws2812Adapter() {
    end();
}

void Ws2812Adapter::begin() {

#ifdef ESP8266
    Serial1.begin(3500000, SERIAL_8N1, SERIAL_TX_ONLY);
    SET_PERI_REG_MASK(UART_CONF0(UART1), BIT22);
#endif
#ifdef ESP32
//    Serial1.begin(3500000, SERIAL_8N1, -1, 23);
//    UART1.conf0.txd_inv = 1; //inverted
#endif

    timer = micros();
}

bool Ws2812Adapter::setBuffer(size_t size) {
    if (bufferSize == size) {
        return true;
    }

    bufferSize = 0;
    buffer.reset(nullptr);
    if (size)
        buffer.reset(new uint8_t[size]);
    if (buffer) {
        bufferSize = size;
        return true;
    }
    return false;
}

void Ws2812Adapter::clearBuffer() {
    setBuffer(0);
}
