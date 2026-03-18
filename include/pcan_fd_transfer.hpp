#pragma once

#include <cstdint>
#include <mutex>

extern "C" {
#include "PCANBasic.h"
}

#include "pcan_long_frame.hpp"
#include "pcan_short_frame.hpp"

class PcanFdTransfer
{
public:
    struct Config
    {
        TPCANHandle handle = PCAN_USBBUS1;

        const char* bitrate_fd =
            "f_clock_mhz=80, "
            "nom_brp=1, nom_tseg1=63, nom_tseg2=16, nom_sjw=16, "
            "data_brp=1, data_tseg1=7, data_tseg2=2, data_sjw=2";

        bool brs_on = true;
    };

    struct RxFrame
    {
        uint32_t can_id = 0u;
        uint8_t data[64] = {0u, };
        uint8_t data_len = 0u;
        bool is_status = false;
        TPCANStatus status = PCAN_ERROR_OK;
    };

    enum class ReadStatus
    {
        Ok,
        Empty,
        Status,
        Error,
    };

    explicit PcanFdTransfer(const Config& cfg,
                            const PcanShortFrameConfig& short_cfg = PcanShortFrameConfig{},
                            const PcanLongFrameConfig& long_cfg = PcanLongFrameConfig{});
    ~PcanFdTransfer();

    bool init(void);
    void shutdown(void);

    /* Compatibility wrappers. New call path should prefer frame objects in handlers. */
    bool send_payload(uint8_t dev_id, uint32_t msg_id, const uint8_t* payload, int payload_len);
    bool send_cmd_with_data(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const uint8_t* payload, uint8_t payload_len);

    /* Low-level RX: read exactly one CAN-FD frame from PCAN queue. */
    ReadStatus read_frame(RxFrame& out);

    const Config& transport_config(void) const { return cfg_; }
    const PcanShortFrameConfig& short_frame_config(void) const { return short_cfg_; }
    const PcanLongFrameConfig& long_frame_config(void) const { return long_cfg_; }

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
    PcanShortFrameConfig short_cfg_;
    PcanLongFrameConfig long_cfg_;
    bool initialized_{false};

    std::mutex io_mtx_;
};
