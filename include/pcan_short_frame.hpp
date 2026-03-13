/**
 * @file    pcan_short_frame.h
 * @author  AU
 * @date    2026.03
 * @brief   Short CAN Frame Handler (PC side)
 *
 * @details
 * Single CAN FD frame command/response processor.
 * Wire format (S32R45 app_can.c 기준):
 *   TX (PC -> S32): [CMD(4B, BE)][DATA ...]
 *   RX (S32 -> PC): [CMD(4B, BE)][UNIQ_ID(4B, BE)][EXTRA_DATA ...]
 *
 * Short frame CAN ID map (app_can.c 기준):
 *   PC  -> S32: tx_base_id + dev_id  (ex: 0x700 + dev)
 *   S32 -> PC : rx_base_id + dev_id  (ex: 0x750 + dev)
 */
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

class PcanFdTransfer;

/* S32R45 command IDs (command_def.h 기준) */
enum class ShortCanCmd : uint32_t
{
    RESET               = 0x10110100u,
    SENSOR_START        = 0x10110203u,
    SENSOR_STOP         = 0x10110302u,
    HI                  = 0x10FF04EBu,
    HEART_BEAT          = 0x10AB4842u,
    TIME_SYNC           = 0x10AB5453u,
    REQUEST_CONNECTION  = 0x41551003u,
};

/*
 * dev_id  : source device index (0 ~ device_count-1)
 * cmd     : received command ID
 * uniq_id : parsed unique ID if present in bytes[4..7], otherwise 0
 * data    : bytes after UNIQ_ID (if present). For simple ACK, this is usually empty.
 */
using ShortFrameRxCallback = std::function<void(
    uint8_t dev_id,
    ShortCanCmd cmd,
    uint32_t uniq_id,
    const std::vector<uint8_t>& data)>;

class PcanShortFrame
{
public:
    struct Config
    {
        uint16_t tx_base_id   = 0x700u;  /* PC  -> S32 */
        uint16_t rx_base_id   = 0x750u;  /* S32 -> PC  */
        uint8_t  device_count = 4u;
    };

    explicit PcanShortFrame(PcanFdTransfer& pcan, const Config& cfg = {});

    bool send_cmd(uint8_t dev_id, ShortCanCmd cmd);
    bool send_cmd_with_data(uint8_t dev_id,
                            ShortCanCmd cmd,
                            const uint8_t* payload,
                            uint8_t payload_len);

    /*
     * S32 update_system_time_from_can() expected payload:
     *   [tv_sec(4B BE)][tv_nsec(4B BE)]
     */
    bool send_time_sync(uint8_t dev_id);

    /*
     * Called by PcanFdTransfer::poll_rx()
     * @return true  : handled as short frame
     *         false : not a short-frame CAN ID
     */
    bool on_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len);

    void set_rx_callback(ShortFrameRxCallback cb);

private:
    bool is_short_rx_can_id(uint32_t can_id, uint8_t& dev_id_out) const;
    void process_short_frame(uint8_t dev_id, const uint8_t* data, uint8_t data_len);

private:
    PcanFdTransfer&     pcan_;
    Config              cfg_;
    ShortFrameRxCallback rx_cb_;
    std::mutex          mtx_;
};
