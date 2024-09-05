#ifndef __UWBIPS_ANCHOR_H__
#define __UWBIPS_ANCHOR_H__

// #ifndef __DEBUGGING__
// #define __DEBUGGING__
// #endif

#ifndef __DEPRECATED__
#define __DEPRECATED__
#endif

#include "interfaces.h"

namespace uwbips_sdk {

    extern dwt_config_t dw3000_config = {
        5,                  // Channel number.
        DWT_PLEN_128,       // Preamble length. Used in TX only.
        DWT_PAC8,           // Preamble acquisition chunk size. Used in RX only.
        9,                  // TX preamble code. Used in TX only.
        9,                  // RX preamble code. Used in RX only.
        1,                  // 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type.
        DWT_BR_6M8,         // Data rate.
        DWT_PHRMODE_STD,    // PHY header mode.
        DWT_PHRRATE_STD,    // PHY header rate.
        (129 + 8 - 8),      // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
        DWT_STS_MODE_OFF,   // STS disabled.
        DWT_STS_LEN_64,     // STS length see allowed values in Enum dwt_sts_lengths_e.
        DWT_PDOA_M0         // PDOA mode off.
    };

    class UWBIPSAnchor {

        public:

            void begin(UWBInfo *inst);

            void setAsTdoaMasterAnchor();

            void setTdoaMasterAnchor(UWBInfo *master);

            void calibrateClockOffset();

            void tdoaExecute(UWBInfo *target);

            uint32_t tdoaReceiveWait();

            void tdoaClockSync();

            double twrExecute(UWBInfo *target);

            void sendTdoaInfoToServer(UWBInfo *info, uint32_t dwtime);

            void sendTwrInfoToServer(UWBInfo *info, double range);

            void receiveServerCommand();

        private:

            UWBInfo inst_info,
                    master_anchor_info;

            bool is_master_anchor = false;

            uint32_t status_reg = 0;
    };
}

#endif