#pragma once

#include <cstdint>
#include <memory>
#include <mutex>

extern "C" {
#include "PCANBasic.h"
}

#include "pcan_long_frame.hpp"
#include "pcan_short_frame.hpp"

class PcanFdTransfer
{
public:
    using LongRxCallback  = PcanLongFrame::LongFrameRxCallback;
    using ShortRxCallback = PcanShortFrame::ShortFrameRxCallback;

    struct Config
    {
        TPCANHandle handle = PCAN_USBBUS1;

        const char* bitrate_fd =
            "f_clock_mhz=80, "
            "nom_brp=1, nom_tseg1=63, nom_tseg2=16, nom_sjw=16, "
            "data_brp=1, data_tseg1=7, data_tseg2=2, data_sjw=2";

        bool brs_on = true;

        /* Long(CustomTP) frame config: CAN IDs, device count, rx buf size, quiet */
        PcanLongFrameConfig  long_frame{};

        /* Short frame config: CAN IDs, device count, quiet */
        PcanShortFrameConfig short_frame{};
    };

    explicit PcanFdTransfer(const Config& cfg);
    ~PcanFdTransfer();

    bool init(void);
    void shutdown(void);

    /* S32 -> PC : drain RX queue and route short/long */
    void poll_rx(void);

    /* RX callback registration */
    void set_long_rx_callback(LongRxCallback cb);
    void set_short_rx_callback(ShortRxCallback cb);

    /* Protocol-layer accessors
     * send_payload()        → PcanLongFrame::send_long_payload()
     * send_cmd_with_data()  → PcanShortFrame::send_short_command_with_data()
     * 상위 핸들러가 직접 호출하도록 레이어를 노출 */
    PcanLongFrame&  long_frame();
    PcanShortFrame& short_frame();

private:
    friend class PcanShortFrame;
    friend class PcanLongFrame;

    static uint8_t len_to_dlc(uint8_t len);
    static uint8_t dlc_to_len(uint8_t dlc);
    static void    print_pcan_err(const char* tag, TPCANStatus st);

    bool send_data(uint16_t can_id, const uint8_t* data, uint8_t length);
    bool send_frame64(uint16_t can_id, const uint8_t data64[64]);

private:
    Config cfg_;
    bool   initialized_{false};

    std::mutex io_mtx_;

    std::unique_ptr<PcanShortFrame> short_frame_;
    std::unique_ptr<PcanLongFrame>  long_frame_;
};
