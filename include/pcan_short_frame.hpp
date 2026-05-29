/**
 * @file    pcan_short_frame.hpp
 * @author  AU
 * @date    2026.03
 * @brief   Short CAN Frame Handler (PC side)
 *
 * @details
 * Single CAN FD frame command/response processor(packing / parsing).
 * Wire format :
 *   TX (PC -> S32): [CMD(4B, BE)][DATA ...]
 *   RX (S32 -> PC): [CMD(4B, BE)][UNIQ_ID(4B, BE)][EXTRA_DATA ...]
 *
 * Short frame CAN ID map :
 *   PC  -> S32: tx_base_id + dev_id  (ex: 0x700 + dev)
 *   S32 -> PC : rx_base_id + dev_id  (ex: 0x750 + dev)
 */
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

class PcanFdTransport;

enum class ShortCanCmd : uint32_t
{
    RESET              = 0x403100u,
    SENSOR_START       = 0x403200u,
    SENSOR_STOP        = 0x403300u,
    HI                 = 0x403400u,
    HEART_BEAT         = 0x404200u,
    TIME_SYNC          = 0x404300u,
    REQUEST_CONNECTION = 0x404400u,
};

struct PcanShortFrameConfig
{
    uint16_t tx_base_id = 0x42u; /* PC -> S32 */
    uint16_t rx_base_id = 0x42u; /* S32 -> PC */
    uint16_t ack_base_id = 0x45u;
    uint8_t device_count = 4u;
    bool quiet = false;
};

class PcanShortFrame
{
public:
    using ShortFrameRxCallback = std::function<void(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& payload)>;
    using AckRxCallback        = std::function<void(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& payload)>;

    using Config = PcanShortFrameConfig;

    explicit PcanShortFrame(PcanFdTransport& transport, const Config& cfg = Config{});

    bool send_short_command(uint8_t dev_id, uint32_t uniq_id, ShortCanCmd cmd);
    bool send_short_command_ack(uint8_t dev_id, uint32_t uniq_id, ShortCanCmd rcv_cmd);
    bool send_short_command_with_data(uint8_t dev_id, uint32_t cmd, uint32_t uniq_id, const uint8_t* payload, uint8_t payload_len);

    bool handle_short_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len);
    bool handle_ack_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len);

    void set_rx_callback(ShortFrameRxCallback cb);
    void set_ack_rx_callback(AckRxCallback cb);

private:
    bool is_short_rx_can_id(uint32_t can_id, uint8_t& dev_id_out) const;
    bool is_ack_rx_can_id(uint32_t can_id, uint8_t& dev_id_out) const;
    void process_short_frame(uint8_t dev_id, const uint8_t* data, uint8_t data_len);
    void process_ack_frame(uint8_t dev_id, const uint8_t* data, uint8_t data_len);

private:
    PcanFdTransport& transport_;
    Config cfg_;
    ShortFrameRxCallback rx_cb_;
    AckRxCallback        ack_rx_cb_;
    std::mutex mtx_;
    static constexpr int CAN_CMD_ACK = 0xAAu;

};
