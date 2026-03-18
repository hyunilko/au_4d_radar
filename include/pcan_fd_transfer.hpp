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
    using LongRxCallback = PcanLongFrame::LongFrameRxCallback;
    using ShortRxCallback = PcanShortFrame::ShortFrameRxCallback;

    struct Config
    {
        TPCANHandle handle = PCAN_USBBUS1;

        const char* bitrate_fd =
            "f_clock_mhz=80, "
            "nom_brp=1, nom_tseg1=63, nom_tseg2=16, nom_sjw=16, "
            "data_brp=1, data_tseg1=7, data_tseg2=2, data_sjw=2";

        uint8_t device_count = 4u;

        /* Short frame CAN IDs */
        uint16_t short_tx_base_id = 0x700u; /* PC -> S32 short */
        uint16_t short_rx_base_id = 0x750u; /* S32 -> PC short */

        /* Long(CustomTP) CAN IDs */
        uint16_t long_tx_base_id = 0x500u;  /* PC -> S32 long */
        uint16_t long_rx_base_id = 0x550u;  /* S32 -> PC long */

        bool brs_on = true;
        bool quiet = false;

        size_t rx_buf_size = 64u * 1024u;
    };

    explicit PcanFdTransfer(const Config& cfg);
    ~PcanFdTransfer();

    bool init(void);
    void shutdown(void);

    /* PC -> S32 : long payload via CustomTP */
    bool send_payload(uint8_t dev_id, uint32_t msg_id, const uint8_t* payload, int payload_len);

    /* PC -> S32 : short frame wrappers */
    bool send_cmd_with_data(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const uint8_t* payload, uint8_t payload_len);


    /* S32 -> PC : drain RX queue and route short/long */
    void poll_rx(void);

    /* RX callback registration */
    void set_long_rx_callback(LongRxCallback cb);
    void set_short_rx_callback(ShortRxCallback cb);

private:
    friend class PcanShortFrame;
    friend class PcanLongFrame;

    static uint8_t len_to_dlc(uint8_t len);
    static uint8_t dlc_to_len(uint8_t dlc);
    static void print_pcan_err(const char* tag, TPCANStatus st);

    bool send_data(uint16_t can_id, const uint8_t* data, uint8_t length);
    bool send_frame64(uint16_t can_id, const uint8_t data64[64]);

private:
    Config cfg_;
    bool initialized_{false};

    std::mutex io_mtx_;

    std::unique_ptr<PcanShortFrame> short_frame_;
    std::unique_ptr<PcanLongFrame> long_frame_;
};
