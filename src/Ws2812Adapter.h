//
// Created by Ben Hencke on 2/10/17.
//

#ifndef WS2812ADAPTER_HPP
#define WS2812ADAPTER_HPP

#include <cstdint>
#include <functional>
#include <memory>

#ifdef ESP32
#include "fastled_compat.h"
#endif

typedef std::function<void(uint16_t index, uint8_t rgb[])> Ws2812PixelFunction;

class Ws2812Adapter {
public:
#ifdef ESP32
    ESP32RMTController mRMTController;
    Ws2812Adapter() : mRMTController(23, C_NS(250), C_NS(500), C_NS(500)) {
        setColorOrder(2, 1, 0);
    };
#else
    Ws2812Adapter(uint8_t o = WS2812_BGR) {
        setColorOrder(o);
    };
#endif

    ~Ws2812Adapter();

    void begin();

    void end();

    void setColorOrder(uint8_t ri, uint8_t gi, uint8_t bi) {
        rOffset = ri;
        gOffset = gi;
        bOffset = bi;
        elements = 3;
    }
    void setColorOrder(uint8_t ri, uint8_t gi, uint8_t bi, uint8_t wi) {
        rOffset = ri;
        gOffset = gi;
        bOffset = bi;
        wOffset = wi;
        elements = 4;
    }

    void show(uint16_t numPixels, Ws2812PixelFunction cb);

private:

    bool setBuffer(size_t size);
    void clearBuffer();

    unsigned long timer;
    uint8_t
            rOffset,                                // Index of red in 3-byte pixel
            gOffset,                                // Index of green byte
            bOffset,                                // Index of blue byte
            wOffset;                                // Index of white byte
    bool useBuffer;
    uint8_t elements = 3; //3 = RGB or 4 = RGBW
    std::unique_ptr<uint8_t[]> buffer;
    size_t bufferSize;

};


#endif //WS2812ADAPTER_HPP
