//
// Created by Ben Hencke on 11/28/17.
//

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


constexpr uint16_t MAX_PULSES = 32;  // A channel has a 64 "pulse" buffer - we use half per pass
constexpr uint16_t DIVIDER    =  4;  // 8 still seems to work, but timings become marginal
constexpr double   RMT_DURATION_NS = 12.5;  // Minimum time of a single RMT duration based on clock ns

constexpr uint32_t  T0H = 270, T1H = 630, T0L = 630, T1L = 270, TRS = 300000;
constexpr rmt_item32_t lowPulse = {
        .level0 = 1, .duration0 = (uint16_t) (T0H / (RMT_DURATION_NS * DIVIDER)),
        .level1 = 0, .duration1 = (uint16_t) (T0L / (RMT_DURATION_NS * DIVIDER)),
        };
constexpr rmt_item32_t highPulse = {
        .level0 = 1, .duration0 = (uint16_t) (T1H / (RMT_DURATION_NS * DIVIDER)),
        .level1 = 0, .duration1 = (uint16_t) (T1L / (RMT_DURATION_NS * DIVIDER)),
};


typedef rmt_item32_t rmtPulsePair;

static intr_handle_t rmt_intr_handle = nullptr;


static IRAM_ATTR void handleRmtInterrupt(void *arg)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

#if DEBUG_ESP32_DIGITAL_LED_LIB
    snprintf(digitalLeds_debugBuffer, digitalLeds_debugBufferSz,
             "%sRMT.int_st.val = %08x\n", digitalLeds_debugBuffer, RMT.int_st.val);
#endif

    for (int i = 0; i < localStrandCnt; i++) {
        strand_t * pStrand = &localStrands[i];
        digitalLeds_stateData * pState = static_cast<digitalLeds_stateData*>(pStrand->_stateVars);

        if (RMT.int_st.val & tx_thr_event_offsets[pStrand->rmtChannel])
        {  // tests RMT.int_st.ch<n>_tx_thr_event
            copyToRmtBlock_half(pStrand);
            RMT.int_clr.val |= tx_thr_event_offsets[pStrand->rmtChannel];  // set RMT.int_clr.ch<n>_tx_thr_event
        }
        else if (RMT.int_st.val & tx_end_offsets[pStrand->rmtChannel] && pState->sem)
        {  // tests RMT.int_st.ch<n>_tx_end and semaphore
            xSemaphoreGiveFromISR(pState->sem, &xHigherPriorityTaskWoken);
            RMT.int_clr.val |= tx_end_offsets[pStrand->rmtChannel];  // set RMT.int_clr.ch<n>_tx_end
            if (xHigherPriorityTaskWoken == pdTRUE)
            {
                portYIELD_FROM_ISR();
            }
        }
    }

    return;
}


void Ws2812Adapter::writeRgb(uint8_t *rgb) {
}

void Ws2812Adapter::show(uint16_t numPixels, Ws2812PixelFunction cb) {
    int curPixel;
    uint8_t rgb[bpp], buf[bpp];

    memset(rgb, 0, bpp);

    if (numPixels > 2500)
        numPixels = 2500;
    setBuffer(numPixels*bpp);
    for (curPixel = 0; curPixel < numPixels; curPixel++) {
        cb(curPixel, rgb);
        buffer[curPixel * bpp] = rgb[0];
        buffer[curPixel * bpp + 1] = rgb[1];
        buffer[curPixel * bpp + 2] = rgb[2];
        if (bpp == 4)
            buffer[curPixel * bpp + 3] = rgb[3];
    }

    //wait for any previous latch
    //TODO can use the rmt peripheral to send reset/latch, just wait for total draw to finish
//    while (micros() - timer < 300) //use ws2813 timing
//        yield();


    //copy pixel stuff, full buffer half way

    RMT.conf_ch[rmtChannel].conf1.mem_rd_rst = 1;
    RMT.conf_ch[rmtChannel].conf1.tx_start = 1;

    //rest handled in interrupt

    //TODO how to handle multiple?


//    for (curPixel = 0; curPixel < numPixels; curPixel++) {
//        //swap around rgb values based on mapping
//        buf[rOffset] = buffer[curPixel * bpp];
//        buf[gOffset] = buffer[curPixel * bpp + 1];
//        buf[bOffset] = buffer[curPixel * bpp + 2];
//        if (bpp == 4)
//            buf[3] = buffer[curPixel * bpp + 3];
//        writeRgb(&buf[0]);
//    }

    timer = micros();
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
#ifdef ESP8266
    Serial1.end();
#endif
}

Ws2812Adapter::~Ws2812Adapter() {
    end();
}

Ws2812Adapter::Ws2812Adapter(uint8_t o) {
    setColorOrder(o);
}



void Ws2812Adapter::setUartFrequency(uint32_t uartFrequency) {

}

void Ws2812Adapter::begin(uint32_t uartFrequency) {
    timer = micros();


    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);

    RMT.apb_conf.fifo_mask = 1;  // Enable memory access, instead of FIFO mode
    RMT.apb_conf.mem_tx_wrap_en = 1;  // Wrap around when hitting end of buffer

    rmt_channel_t rmtChannel = RMT_CHANNEL_0;

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

    RMT.int_ena.val |= 1 << (24 + rmtChannel); //threshold interrupt ch<X>_tx_thr_event
    RMT.int_ena.val |= 1 << rmtChannel * 3; // end interrupt ch<X>_tx_end

    rmt_config()

    esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, handleRmtInterrupt, nullptr, &rmt_intr_handle);

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

}




