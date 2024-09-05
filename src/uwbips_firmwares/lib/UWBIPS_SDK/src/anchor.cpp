#include "anchor.h"



dwt_config_t uwbips_sdk::dw3000_config;



void uwbips_sdk::UWBIPSAnchor::begin(uwbips_sdk::UWBInfo *inst) {

    #ifdef __DEBUGGING__
        Serial.begin(115200);
    #endif

    memcpy(&this->inst_info, inst, sizeof(uwbips_sdk::UWBInfo));

    if(this->inst_info.dwtype == uwbips_sdk::DWTYPE_DW1000) {

    }

    else if(this->inst_info.dwtype == uwbips_sdk::DWTYPE_DW1000_EXTREME) {
        
    }

    else if(this->inst_info.dwtype == uwbips_sdk::DWTYPE_DW3000) {
    
        extern SPISettings _fastSPI;
        _fastSPI = SPISettings(
            16000000L, 
            MSBFIRST, 
            SPI_MODE0
        );
        spiBegin(PIN_IRQ, PIN_RST);
        spiSelect(PIN_CS);
        delay(2);

        while(!dwt_checkidlerc());
        while(dwt_initialise(DWT_DW_INIT) == DWT_ERROR);

        extern dwt_txconfig_t txconfig_options;
        dwt_configure(&uwbips_sdk::dw3000_config);
        dwt_configuretxrf(&txconfig_options);

        dwt_setrxantennadelay(16385);
        dwt_settxantennadelay(16385);

        dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
        dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    }
}



void uwbips_sdk::UWBIPSAnchor::setAsTdoaMasterAnchor() {

    this->is_master_anchor = true;
}



void uwbips_sdk::UWBIPSAnchor::setTdoaMasterAnchor(uwbips_sdk::UWBInfo *master) {

    memcpy(&this->master_anchor_info, master, sizeof(uwbips_sdk::UWBInfo));
}



void uwbips_sdk::UWBIPSAnchor::calibrateClockOffset() {

}



void uwbips_sdk::UWBIPSAnchor::tdoaExecute(uwbips_sdk::UWBInfo *target) {

}



uint32_t uwbips_sdk::UWBIPSAnchor::tdoaReceiveWait() {

    if(this->inst_info.dwtype == uwbips_sdk::DWTYPE_DW1000) {

    }

    else if(this->inst_info.dwtype == uwbips_sdk::DWTYPE_DW1000_EXTREME) {

    }

    else if(this->inst_info.dwtype == uwbips_sdk::DWTYPE_DW3000) {
        
        uint32_t status_reg = 0;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        while(!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)));
    }
}



void uwbips_sdk::UWBIPSAnchor::tdoaClockSync() {

}



double uwbips_sdk::UWBIPSAnchor::twrExecute(uwbips_sdk::UWBInfo *target) {

}



void uwbips_sdk::UWBIPSAnchor::sendTdoaInfoToServer(uwbips_sdk::UWBInfo *info, uint32_t dwtime) {

}



void uwbips_sdk::UWBIPSAnchor::sendTwrInfoToServer(uwbips_sdk::UWBInfo *info, double range) {

}



void uwbips_sdk::UWBIPSAnchor::receiveServerCommand() {

}




