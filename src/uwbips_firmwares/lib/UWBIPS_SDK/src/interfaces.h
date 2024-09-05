#ifndef __UWBIPS_INTERFACES_H__
#define __UWBIPS_INTERFACES_H__

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <esp_now.h>
#include <SPI.h>
#include "dw3000.h"
#include "DW1000.h"

namespace uwbips_sdk {

    enum DWType : uint8_t {
        DWTYPE_DW1000,
        DWTYPE_DW1000_EXTREME,
        DWTYPE_DW3000
    };

    enum UWBMode : uint8_t {
        MODE_ANCHOR,
        MODE_MASTER_ANCHOR,
        MODE_TAG_TDOA,
        MODE_TAG_TWR
    };

    enum UWBPin : uint8_t {
        PIN_RST     = 27,
        PIN_IRQ     = 34,
        PIN_SS      = 4,
        PIN_SCK     = 18,
        PIN_MISO    = 19,
        PIN_MOSI    = 23,
        PIN_CS      = 4
    };

    struct UWBInfo {
        char*   address;
        DWType  dwtype;
        UWBMode mode;
    };
};

#endif