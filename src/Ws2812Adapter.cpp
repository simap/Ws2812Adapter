//
// Created by Ben Hencke on 11/28/17.
//

/*
 * Based on
 * Library for driving digital RGB(W) LEDs using the ESP32's RMT peripheral
 *
 * Modifications Copyright (c) 2017 Martin F. Falatic
 *
 * Based on public domain code created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 */
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


#if defined(ARDUINO)

#include <Arduino.h>
#include "esp32-hal.h"
#include "esp_intr.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "freertos/semphr.h"
#include "soc/rmt_struct.h"

#elif defined(ESP_PLATFORM)
#include <esp_intr.h>
#include <driver/gpio.h>
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <soc/dport_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/rmt_struct.h>
#include <stdio.h>
#include <string.h>  // memset, memcpy, etc. live here!
#endif



// -- Configuration constants
#define DIVIDER             2 /* 4, 8 still seem to work, but timings become marginal */
// -- Convert ESP32 cycles back into nanoseconds
#define ESPCLKS_TO_NS(_CLKS) (((long)(_CLKS) * 1000L) / F_CPU_MHZ)
// -- Convert nanoseconds into RMT cycles
#define F_CPU_RMT       (  80000000L)
#define NS_PER_SEC      (1000000000L)
#define CYCLES_PER_SEC  (F_CPU_RMT/DIVIDER)
#define NS_PER_CYCLE    ( NS_PER_SEC / CYCLES_PER_SEC )
#define NS_TO_CYCLES(n) ( (n) / NS_PER_CYCLE )
// -- Convert ESP32 cycles to RMT cycles
#define TO_RMT_CYCLES(_CLKS) NS_TO_CYCLES(ESPCLKS_TO_NS(_CLKS))



constexpr uint16_t MAX_PULSES = 32;  // A channel has a 64 "pulse" buffer - we use half per pass
constexpr double RMT_DURATION_NS = 12.5;  // Minimum time of a single RMT duration based on clock ns

constexpr uint32_t T0H = 270, T1H = 630, T0L = 630, T1L = 270, TRS = 300000;
constexpr uint16_t TRS_DURATION = (uint16_t) NS_TO_CYCLES(300000); //(TRS / (RMT_DURATION_NS * DIVIDER));

// LUT for mapping bits in RMT.int_<op>.ch<n>_tx_thr_event
static DRAM_ATTR const uint32_t tx_thr_event_offsets[] = {
        static_cast<uint32_t>(1) << (24 + 0),
        static_cast<uint32_t>(1) << (24 + 1),
        static_cast<uint32_t>(1) << (24 + 2),
        static_cast<uint32_t>(1) << (24 + 3),
        static_cast<uint32_t>(1) << (24 + 4),
        static_cast<uint32_t>(1) << (24 + 5),
        static_cast<uint32_t>(1) << (24 + 6),
        static_cast<uint32_t>(1) << (24 + 7),
};

// LUT for mapping bits in RMT.int_<op>.ch<n>_tx_end
static DRAM_ATTR const uint32_t tx_end_offsets[] = {
        static_cast<uint32_t>(1) << (0 + 0) * 3,
        static_cast<uint32_t>(1) << (0 + 1) * 3,
        static_cast<uint32_t>(1) << (0 + 2) * 3,
        static_cast<uint32_t>(1) << (0 + 3) * 3,
        static_cast<uint32_t>(1) << (0 + 4) * 3,
        static_cast<uint32_t>(1) << (0 + 5) * 3,
        static_cast<uint32_t>(1) << (0 + 6) * 3,
        static_cast<uint32_t>(1) << (0 + 7) * 3,
};

constexpr rmt_channel_t rmtChannel = RMT_CHANNEL_0;

typedef rmt_item32_t rmtPulsePair;

static intr_handle_t rmt_intr_handle = nullptr;


static IRAM_ATTR void handleRmtInterrupt(void *arg) {
    Ws2812Adapter *adapter = (Ws2812Adapter *) arg;

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if (RMT.int_st.val & tx_thr_event_offsets[rmtChannel]) {  // tests RMT.int_st.ch<n>_tx_thr_event
        adapter->copyToRmtBlock_half();
        RMT.int_clr.val |= tx_thr_event_offsets[rmtChannel];  // set RMT.int_clr.ch<n>_tx_thr_event
    } else if (RMT.int_st.val & tx_end_offsets[rmtChannel] &&
            adapter->drawSem) {  // tests RMT.int_st.ch<n>_tx_end and semaphore
//        Serial.println("unlock from isr");
        xSemaphoreGiveFromISR(adapter->drawSem, &xHigherPriorityTaskWoken);
        RMT.int_clr.val |= tx_end_offsets[rmtChannel];  // set RMT.int_clr.ch<n>_tx_end
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}


void Ws2812Adapter::writeRgb(uint8_t *rgb) {
}

void Ws2812Adapter::show(uint16_t numPixels, Ws2812PixelFunction cb) {
    int curPixel;
    uint8_t rgb[bpp], buf[bpp];

    memset(rgb, 0, bpp);

//    if (numPixels > 2500)
//        numPixels = 2500;
    setBuffer(numPixels * bpp);
    for (curPixel = 0; curPixel < numPixels; curPixel++) {
        cb(curPixel, rgb);
        int pixelOffset = curPixel * bpp;
        buffer[pixelOffset + rOffset] = rgb[0];
        buffer[pixelOffset + gOffset] = rgb[1];
        buffer[pixelOffset + bOffset] = rgb[2];
        if (bpp == 4)
            buffer[pixelOffset + 3] = rgb[3];
    }

    //wait for any previous latch
    //TODO can use the rmt peripheral to send reset/latch, just wait for total draw to finish
//    while (micros() - timer < 300) //use ws2813 timing
//        yield();

//    Serial.print("Waiting for lock ");
    // block until current draw is complete
    if (!drawSem || !xSemaphoreTake(drawSem, portMAX_DELAY))
        return;

//    Serial.println("locked!");

    buf_pos = 0;
    buf_half = 0;
    buf_len = numPixels * bpp;

    copyToRmtBlock_half();
    if (buf_pos < buf_len) {
        // Fill the other half of the buffer block
        copyToRmtBlock_half();
    }

    RMT.conf_ch[rmtChannel].conf1.mem_rd_rst = 1;
    RMT.conf_ch[rmtChannel].conf1.tx_start = 1;

    //rest handled in interrupt

}

void Ws2812Adapter::setColorOrder(uint8_t o) {
    rOffset = (uint8_t) ((o & 3));
    gOffset = (uint8_t) (((o >> 2) & 3));
    bOffset = (uint8_t) (((o >> 4) & 3));
}

void Ws2812Adapter::setColorOrder(uint8_t o, bool hasWhite) {
    setColorOrder(o);
    bpp = hasWhite ? 4 : 3;
}


void Ws2812Adapter::end() {
    //wait for any draw operation to finish
    if (!drawSem || !xSemaphoreTake(drawSem, portMAX_DELAY))
        return;
    xSemaphoreGive(drawSem);
}

Ws2812Adapter::~Ws2812Adapter() {
    end();
}

Ws2812Adapter::Ws2812Adapter(uint8_t o) {
    setColorOrder(o);
    drawSem = xSemaphoreCreateBinary();
    xSemaphoreGive(drawSem);

    lowPulse.level0 = 1;
    lowPulse.duration0 = (uint16_t) NS_TO_CYCLES(T0H);
    lowPulse.level1 = 0;
    lowPulse.duration1 = (uint16_t) NS_TO_CYCLES(T0L);

    highPulse.level0 = 1;
    highPulse.duration0 = (uint16_t) NS_TO_CYCLES(T1H);
    highPulse.level1 = 0;
    highPulse.duration1 = (uint16_t) NS_TO_CYCLES (T1L);

}


void Ws2812Adapter::setUartFrequency(uint32_t uartFrequency) {

}

void Ws2812Adapter::begin(uint32_t uartFrequency) {
    timer = micros();


    //could maybe hold RMT in reset, or disable clock to set up for parallel output?

    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);

    RMT.apb_conf.fifo_mask = 1;  // Enable memory access, instead of FIFO mode
    RMT.apb_conf.mem_tx_wrap_en = 1;  // Wrap around when hitting end of buffer


    rmt_set_pin(rmtChannel, RMT_MODE_TX, GPIO_NUM_23);


    RMT.conf_ch[rmtChannel].conf0.div_cnt = DIVIDER;
    RMT.conf_ch[rmtChannel].conf0.mem_size = 1;
    RMT.conf_ch[rmtChannel].conf0.carrier_en = 0;
    RMT.conf_ch[rmtChannel].conf0.carrier_out_lv = 1;
    RMT.conf_ch[rmtChannel].conf0.mem_pd = 0;

    RMT.conf_ch[rmtChannel].conf1.rx_en = 0;
    RMT.conf_ch[rmtChannel].conf1.mem_owner = 0;
    RMT.conf_ch[rmtChannel].conf1.tx_conti_mode = 0;  //loop back mode
    RMT.conf_ch[rmtChannel].conf1.ref_always_on = 1;  // use apb clock: 80M
    RMT.conf_ch[rmtChannel].conf1.idle_out_en = 1;
    RMT.conf_ch[rmtChannel].conf1.idle_out_lv = 0;

    RMT.tx_lim_ch[rmtChannel].limit = MAX_PULSES;

    RMT.int_ena.val |= tx_thr_event_offsets[rmtChannel];  // RMT.int_ena.ch<n>_tx_thr_event = 1;
    RMT.int_ena.val |= tx_end_offsets[rmtChannel];  // RMT.int_ena.ch<n>_tx_end = 1;

    esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, handleRmtInterrupt, this, &rmt_intr_handle);

}

bool Ws2812Adapter::setBuffer(size_t size) {
    if (bufferSize == size) {
        return true;
    }
    // block until current draw is complete if we need to resize
    if (!drawSem || !xSemaphoreTake(drawSem, portMAX_DELAY))
        return false;
    xSemaphoreGive(drawSem);

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

IRAM_ATTR void Ws2812Adapter::copyToRmtBlock_half() {
    // This fills half an RMT block
    // When wraparound is happening, we want to keep the inactive half of the RMT block filled

    uint16_t i, j, offset, len, byteval;

    offset = buf_half ? MAX_PULSES : 0;
    buf_half = !buf_half;

    len = buf_len - buf_pos;
    if (len > (MAX_PULSES / 8))
        len = (MAX_PULSES / 8);

    if (!len) {
        if (!buf_isDirty) {
            return;
        }
        // Clear the channel's data block and return
        for (i = 0; i < MAX_PULSES; i++) {
            RMTMEM.chan[rmtChannel].data32[i + offset].val = 0;
        }
        buf_isDirty = false;
        return;
    }
    buf_isDirty = true;

    for (i = 0; i < len; i++) {
        byteval = buffer[i + buf_pos];

#if DEBUG_ESP32_DIGITAL_LED_LIB
        snprintf(digitalLeds_debugBuffer, digitalLeds_debugBufferSz,
               "%s%d(", digitalLeds_debugBuffer, byteval);
#endif


        volatile uint32_t *dst = &RMTMEM.chan[rmtChannel].data32[i * 8 + offset].val;

        // Shift bits out, MSB first, setting RMTMEM.chan[n].data32[x] to
        // the rmtPulsePair value corresponding to the buffered bit value
        for (j = 0; j < 8; j++, byteval <<= 1) {
            int bitval = (byteval >> 7) & 0x01;
            int data32_idx = i * 8 + offset + j;
            RMTMEM.chan[rmtChannel].data32[data32_idx].val = bitval ? highPulse.val
                                                                    : lowPulse.val; //pulsePairMap[bitval].val;
#if DEBUG_ESP32_DIGITAL_LED_LIB
            snprintf(digitalLeds_debugBuffer, digitalLeds_debugBufferSz,
                 "%s%d", digitalLeds_debugBuffer, bitval);
#endif
        }
#if DEBUG_ESP32_DIGITAL_LED_LIB
        snprintf(digitalLeds_debugBuffer, digitalLeds_debugBufferSz,
               "%s) ", digitalLeds_debugBuffer);
#endif

        // Handle the reset bit by stretching duration1 for the final bit in the stream
        if (i + buf_pos == buf_len - 1) {
            //TODO how to ensure this is always last? what if data evenly fits?
            RMTMEM.chan[rmtChannel].data32[i * 8 + offset + 7].duration1 = TRS_DURATION;
#if DEBUG_ESP32_DIGITAL_LED_LIB
            snprintf(digitalLeds_debugBuffer, digitalLeds_debugBufferSz,
                 "%sRESET ", digitalLeds_debugBuffer);
#endif
        }
    }

    // Clear the remainder of the channel's data not set above
    for (i *= 8; i < MAX_PULSES; i++) {
        RMTMEM.chan[rmtChannel].data32[i + offset].val = 0;
    }

    buf_pos += len;

#if DEBUG_ESP32_DIGITAL_LED_LIB
    snprintf(digitalLeds_debugBuffer, digitalLeds_debugBufferSz,
             "%s ", digitalLeds_debugBuffer);
#endif

    return;
}



