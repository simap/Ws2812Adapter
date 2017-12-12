//
// Created by Ben Hencke on 11/28/17.
//

#include "Ws2812Adapter.h"

//TODO switch back and forth between slow and fast mode
//static const uint8_t bits[4] = {
//        0b110111,
//        0b000111,
//        0b110100,
//        0b000100,
//};

void Ws2812Adapter::writeRgb(uint8_t *rgb) {
//    for (uint8_t i = 0; i < 3; i++) {
//        uint8_t c = rgb[i];
//        Serial1.write(bits[c >> 6]);
//        Serial1.write(bits[(c >> 4) & 0x03]);
//        Serial1.write(bits[(c >> 2) & 0x03]);
//        Serial1.write(bits[c & 0x03]);
//    }

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
}

void Ws2812Adapter::show(uint16_t numPixels, Ws2812PixelFunction cb) {
    int curPixel;

    uint8_t rgb[3], buf[3];

    //wait for any previous latch
    while (micros() - timer < 300) //use ws2813 timing
        yield();

    //pixels, sourced from callback
    for (curPixel = 0; curPixel < numPixels; curPixel++) {
        cb(curPixel, rgb);

        //swap around rgb values based on mapping
        buf[rOffset] = rgb[0];
        buf[gOffset] = rgb[1];
        buf[bOffset] = rgb[2];
        writeRgb(&buf[0]);
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

void Ws2812Adapter::begin() {
//    Serial1.begin(3200000, SERIAL_6N1, SERIAL_TX_ONLY);
    Serial1.begin(2000000, SERIAL_8N1, SERIAL_TX_ONLY);

    SET_PERI_REG_MASK(UART_CONF0(UART1), BIT22);
    timer = micros();
}
