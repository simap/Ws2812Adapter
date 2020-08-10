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

//borrowed from Adafruit
// Color-order flag for LED pixels (optional extra parameter to constructor):
// Bits 0,1 = R index (0-2), bits 2,3 = G index, bits 4,5 = B index
#define WS2812_RGB (0 | (1 << 2) | (2 << 4))
#define WS2812_RBG (0 | (2 << 2) | (1 << 4))
#define WS2812_GRB (1 | (0 << 2) | (2 << 4))
#define WS2812_GBR (2 | (0 << 2) | (1 << 4))
#define WS2812_BRG (1 | (2 << 2) | (0 << 4))
#define WS2812_BGR (2 | (1 << 2) | (0 << 4))

typedef std::function<void(uint16_t index, uint8_t rgb[])> Ws2812PixelFunction;



class Ws2812Adapter {
public:
#ifdef ESP32
    ESP32RMTController mRMTController;
    Ws2812Adapter(uint8_t o = WS2812_BGR) : mRMTController(23, C_NS(250), C_NS(625), C_NS(375)) {
        setColorOrder(o);
    };
#else
    Ws2812Adapter(uint8_t o = WS2812_BGR) {
        setColorOrder(o);
    };
#endif

    ~Ws2812Adapter();

    void begin();

    void end();

    void setColorOrder(uint8_t o);
    void setColorOrder(uint8_t o, bool hasWhite);

    void show(uint16_t numPixels, Ws2812PixelFunction cb);

private:

    bool setBuffer(size_t size);
    void clearBuffer();

    unsigned long timer;
    uint8_t
            rOffset,                                // Index of red in 3-byte pixel
            gOffset,                                // Index of green byte
            bOffset;                                // Index of blue byte
    bool useBuffer;
    bool hasWhite;
    uint8_t bpp = 3;
    std::unique_ptr<uint8_t[]> buffer;
    size_t bufferSize;

};


#endif //WS2812ADAPTER_HPP
