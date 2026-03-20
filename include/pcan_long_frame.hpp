/**
 * @file    pcan_long_frame.hpp
 * @author  AU
 * @date    2026.03
 * @brief   long CAN Frame Handler (PC side)
 *
 * @details
 * CustomTP reassembler/transmitter for large CAN-FD payloads.
 * long frame CAN ID map :
 *   PC  -> S32: tx_base_id + dev_id  (ex: 0x500 + dev)
 *   S32 -> PC : rx_base_id + dev_id  (ex: 0x550 + dev)
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <vector>

class PcanFdTransfer;

struct PcanLongFrameConfig
{
    uint16_t tx_base_id = 0x500u; /* PC -> S32 long */
    uint16_t rx_base_id = 0x550u; /* S32 -> PC long */
    uint8_t device_count = 4u;
    size_t rx_buf_size = 64u * 1024u;
    bool quiet = false;
};

class PcanLongFrame
{
public:
    using LongFrameRxCallback = std::function<void(uint8_t dev_id, uint32_t frame_id, uint32_t frame_count, uint32_t msg_id, std::vector<uint8_t>&& payload)>;

    using Config = PcanLongFrameConfig;

    explicit PcanLongFrame(PcanFdTransfer& transport, const Config& cfg = Config{});

    bool send_long_payload(uint8_t dev_id, uint32_t msg_id, const uint8_t* payload, int payload_len);
    bool handle_long_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len);

    void set_rx_callback(LongFrameRxCallback cb);

private:
    struct RxState
    {
        std::vector<uint8_t> buf;
        uint32_t len = 0u;
        uint16_t seq_expect = 0u;

        explicit RxState(size_t cap) : buf(cap, 0u) {}

        void reset(void)
        {
            len = 0u;
            seq_expect = 0u;
            std::fill(buf.begin(), buf.end(), 0u);
        }
    };

    bool is_long_rx_can_id(uint32_t can_id, uint8_t& dev_id_out) const;
    void process_long_tp_frame(uint8_t dev_id, const uint8_t* data, uint8_t data_len);

private:
    PcanFdTransfer& transport_;
    Config cfg_;
    LongFrameRxCallback rx_cb_;
    std::vector<RxState> rx_states_;
    std::vector<uint32_t> tx_frame_count_;
};
