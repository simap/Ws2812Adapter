//
// Created by Ben Hencke on 11/28/17.
//

#include "Ws2812Adapter.h"
#include "../../../../../.platformio/packages/toolchain-xtensa/xtensa-lx106-elf/include/c++/4.8.2/cstdio"
#include <cstdlib>

//TODO switch back and forth between slow and fast mode
static const uint8_t bits[4] = {
        0b11101111,
        0b11001111,
        0b11101110,
        0b11001110,
};

void Ws2812Adapter::writeRgb(uint8_t *rgb) {
    if (useBuffer) {
        //wait for 24 bytes free in the uart tx fifo
        while((USS(UART1) >> USTXC) >= 116);
        os_intr_lock();
        for (uint8_t i = 0; i < 3; i++) {
            uint8_t c = rgb[i];
            Serial1.write(bits[c >> 6]);
            Serial1.write(bits[(c >> 4) & 0x03]);
            Serial1.write(bits[(c >> 2) & 0x03]);
            Serial1.write(bits[c & 0x03]);
        }
        os_intr_unlock();
        return;
    }

    //wait for 24 bytes free in the uart tx fifo
    while((USS(UART1) >> USTXC) >= 104);

    os_intr_lock();
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t m = 0x80;
        for (uint8_t b = 0; b < 8; b++) {
            int tmp = (rgb[i] & m) ? 0b11111110 : 0b11111111;
//            if (tmp != 0b11111111)
//                Serial.println(rgb[i]);
            Serial1.write(tmp);
            m >>= 1;
        }
    }
    os_intr_unlock();
}

void Ws2812Adapter::show(uint16_t numPixels, Ws2812PixelFunction cb) {
    int curPixel;
    uint8_t rgb[3], buf[3];

    if (useBuffer) {
        if (numPixels > 2500)
            numPixels = 2500;
        setBuffer(numPixels*3);
        for (curPixel = 0; curPixel < numPixels; curPixel++) {
            cb(curPixel, rgb);
            buffer[curPixel * 3] = rgb[0];
            buffer[curPixel * 3 + 1] = rgb[1];
            buffer[curPixel * 3 + 2] = rgb[2];
        }
    }

    //wait for any previous latch
    while (micros() - timer < 300) //use ws2813 timing
        yield();

    if (useBuffer) {
        for (curPixel = 0; curPixel < numPixels; curPixel++) {
            //swap around rgb values based on mapping
            buf[rOffset] = buffer[curPixel * 3];
            buf[gOffset] = buffer[curPixel * 3 + 1];
            buf[bOffset] = buffer[curPixel * 3 + 2];
            writeRgb(&buf[0]);
        }
    } else {
        //pixels, sourced from callback
        for (curPixel = 0; curPixel < numPixels; curPixel++) {
            cb(curPixel, rgb);

            //swap around rgb values based on mapping
            buf[rOffset] = rgb[0];
            buf[gOffset] = rgb[1];
            buf[bOffset] = rgb[2];
            writeRgb(&buf[0]);
        }
    }

    Serial1.flush();
    timer = micros();
}

void Ws2812Adapter::setColorOrder(uint8_t o) {
    rOffset = (uint8_t) ((o & 3));
    gOffset = (uint8_t) (((o >> 2) & 3));
    bOffset = (uint8_t) (((o >> 4) & 3));
}

void Ws2812Adapter::end() {
    Serial1.end();
}

Ws2812Adapter::~Ws2812Adapter() {
    end();
}

Ws2812Adapter::Ws2812Adapter(uint8_t o) {
    setColorOrder(o);
}

void Ws2812Adapter::setUartFrequency(uint32_t uartFrequency) {
    Serial1.begin(uartFrequency, SERIAL_8N1, SERIAL_TX_ONLY);

    SET_PERI_REG_MASK(UART_CONF0(UART1), BIT22);
}

void Ws2812Adapter::begin(uint32_t uartFrequency) {
    setUartFrequency(uartFrequency);
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

void Ws2812Adapter::setUseBuffer(bool newUseBuffer) {
    useBuffer = newUseBuffer;
    if (!useBuffer)
        clearBuffer();
}



