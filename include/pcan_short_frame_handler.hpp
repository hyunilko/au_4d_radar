#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "rclcpp/logger.hpp"

#include "pcan_fd_transfer.hpp"
#include "pcan_short_frame.hpp"

namespace au_4d_radar
{

class device_au_radar_node;

class PcanShortFrameHandler
{
public:
    struct AckMessage
    {
        uint8_t dev_id = 0u;
        ShortCanCmd cmd = ShortCanCmd::HI;
        uint32_t uniq_id = 0u;
        std::vector<uint8_t> payload;
        std::chrono::steady_clock::time_point rx_time{};
    };

    using AckCallback = std::function<void(const AckMessage& msg)>;

    explicit PcanShortFrameHandler(device_au_radar_node* node,
                                   PcanFdTransfer& can,
                                   rclcpp::Logger logger = rclcpp::get_logger("PcanShortFrameHandler"),
                                   bool quiet = false);

    void start(void);
    void stop(void);

    bool handle_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len);

    void set_ack_callback(AckCallback cb);
    bool wait_for_ack(uint8_t dev_id, ShortCanCmd cmd, AckMessage& out,
                      std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

    bool get_device_uniq_id(uint8_t dev_id, uint32_t& uniq_id_out) const;

private:
    using AckKey = std::uint64_t;

    static AckKey make_ack_key(uint8_t dev_id, ShortCanCmd cmd);
    void handle_cmd_ack(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& data);
    void handle_short_frame(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& data);
    bool send_time_sync(uint8_t dev_id, uint32_t uniq_id);

    PcanFdTransfer& can_;
    PcanShortFrame short_frame_;
    rclcpp::Logger logger_;
    bool quiet_ = false;
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    AckCallback ack_cb_;
    std::unordered_map<AckKey, AckMessage> ack_map_;
    std::unordered_map<uint8_t, uint32_t> uniq_id_map_;
    device_au_radar_node* radar_node_;
};

} // namespace au_4d_radar
