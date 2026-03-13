#include "pcan_short_frame_handler.hpp"

#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace au_4d_radar
{

PcanShortFrameHandler::PcanShortFrameHandler(device_au_radar_node* node, 
                                             PcanFdTransfer& can,
                                             rclcpp::Logger logger,
                                             bool quiet)
    : can_(can)
    , logger_(std::move(logger))
    , quiet_(quiet)
    , radar_node_(node)
{
}

void PcanShortFrameHandler::start(void)
{
    can_.set_short_rx_callback(
        [this](uint8_t dev_id,
               ShortCanCmd cmd,
               uint32_t uniq_id,
               const std::vector<uint8_t>& data)
        {
            this->handle_short_ack(dev_id, cmd, uniq_id, data);
        });

    if (!quiet_)
    {
        RCLCPP_INFO(logger_, "PcanShortFrameHandler started");
    }
}

void PcanShortFrameHandler::stop(void)
{
    can_.set_short_rx_callback(ShortFrameRxCallback{});

    std::lock_guard<std::mutex> lk(mtx_);
    ack_cb_ = AckCallback{};
    ack_map_.clear();
    uniq_id_map_.clear();

    if (!quiet_)
    {
        RCLCPP_INFO(logger_, "PcanShortFrameHandler stopped");
    }
}

void PcanShortFrameHandler::set_ack_callback(AckCallback cb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    ack_cb_ = std::move(cb);
}

bool PcanShortFrameHandler::send_hi(uint8_t dev_id)
{
    return can_.send_short_cmd(dev_id, ShortCanCmd::HI);
}

bool PcanShortFrameHandler::send_request_connection(uint8_t dev_id)
{
    return can_.send_short_cmd(dev_id, ShortCanCmd::REQUEST_CONNECTION);
}

bool PcanShortFrameHandler::send_time_sync(uint8_t dev_id)
{
    return can_.send_time_sync(dev_id);
}

bool PcanShortFrameHandler::send_sensor_start(uint8_t dev_id,
                                              const uint8_t* payload,
                                              uint8_t payload_len)
{
    return can_.send_short_cmd_with_data(dev_id, ShortCanCmd::SENSOR_START, payload, payload_len);
}

bool PcanShortFrameHandler::send_sensor_stop(uint8_t dev_id,
                                             const uint8_t* payload,
                                             uint8_t payload_len)
{
    return can_.send_short_cmd_with_data(dev_id, ShortCanCmd::SENSOR_STOP, payload, payload_len);
}

bool PcanShortFrameHandler::send_reset(uint8_t dev_id,
                                       const uint8_t* payload,
                                       uint8_t payload_len)
{
    return can_.send_short_cmd_with_data(dev_id, ShortCanCmd::RESET, payload, payload_len);
}

bool PcanShortFrameHandler::wait_for_ack(uint8_t dev_id,
                                         ShortCanCmd cmd,
                                         AckMessage& out,
                                         std::chrono::milliseconds timeout)
{
    const AckKey key = make_ack_key(dev_id, cmd);

    std::unique_lock<std::mutex> lk(mtx_);
    const bool ok = cv_.wait_for(
        lk,
        timeout,
        [this, key]()
        {
            return ack_map_.find(key) != ack_map_.end();
        });

    if (!ok)
    {
        if (!quiet_)
        {
            RCLCPP_WARN(logger_,
                        "wait_for_ack timeout dev=%u cmd=0x%08X timeout_ms=%lld",
                        dev_id,
                        static_cast<uint32_t>(cmd),
                        static_cast<long long>(timeout.count()));
        }
        return false;
    }

    out = ack_map_.at(key);
    return true;
}

bool PcanShortFrameHandler::get_device_uniq_id(uint8_t dev_id, uint32_t& uniq_id_out) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    const auto it = uniq_id_map_.find(dev_id);
    if (it == uniq_id_map_.end())
    {
        return false;
    }

    uniq_id_out = it->second;
    return true;
}

PcanShortFrameHandler::AckKey PcanShortFrameHandler::make_ack_key(uint8_t dev_id, ShortCanCmd cmd)
{
    return (static_cast<AckKey>(dev_id) << 32) | static_cast<uint32_t>(cmd);
}

void PcanShortFrameHandler::handle_short_ack(uint8_t dev_id,
                                             ShortCanCmd cmd,
                                             uint32_t uniq_id,
                                             const std::vector<uint8_t>& data)
{
    AckMessage msg;
    msg.dev_id = dev_id;
    msg.cmd = cmd;
    msg.uniq_id = uniq_id;
    msg.extra = data;
    msg.rx_time = std::chrono::steady_clock::now();

    AckCallback cb_copy;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        ack_map_[make_ack_key(dev_id, cmd)] = msg;
        if (uniq_id != 0u)
        {
            uniq_id_map_[dev_id] = uniq_id;
        }
        cb_copy = ack_cb_;
    }

    cv_.notify_all();

    if (!quiet_)
    {
        RCLCPP_INFO(logger_,
                    "[SHORT ACK] dev=%u cmd=0x%08X uniq_id=0x%08X extra_len=%zu",
                    dev_id,
                    static_cast<uint32_t>(cmd),
                    uniq_id,
                    data.size());
    }

    if (cb_copy)
    {
        cb_copy(msg);
    }
}

}  // namespace au_4d_radar