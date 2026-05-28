#pragma once
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "Hardware.h"

// LovyanGFX panel config lifted verbatim from Elecrow's RotaryScreen_1_28_new.ino
// — the known-good driver setup for the GC9A01 on this exact board.
class CrowPanelLGFX : public lgfx::LGFX_Device {
    lgfx::Panel_GC9A01 _panel;
    lgfx::Bus_SPI _bus;
public:
    CrowPanelLGFX() {
        {
            auto cfg = _bus.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 80000000;
            cfg.freq_read = 20000000;
            cfg.spi_3wire = true;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = HW::PIN_DISP_SCLK;
            cfg.pin_mosi = HW::PIN_DISP_MOSI;
            cfg.pin_miso = -1;
            cfg.pin_dc   = HW::PIN_DISP_DC;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg = _panel.config();
            cfg.pin_cs   = HW::PIN_DISP_CS;
            cfg.pin_rst  = HW::PIN_DISP_RST;
            cfg.pin_busy = -1;
            cfg.memory_width  = 240;
            cfg.memory_height = 240;
            cfg.panel_width   = 240;
            cfg.panel_height  = 240;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits = 1;
            cfg.readable = false;
            cfg.invert = true;
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = false;
            _panel.config(cfg);
        }
        setPanel(&_panel);
    }
};

namespace DisplayHW {
    // PWM channel/freq for the backlight pin.
    constexpr int BL_LEDC_FREQ = 5000;
    constexpr int BL_LEDC_RESOLUTION_BITS = 8;
    constexpr int BL_DEFAULT_PCT = 70;
}
