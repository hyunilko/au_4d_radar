#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "util/conversion.hpp"
#include "pcan_short_frame_handler.hpp"

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
    can_.set_short_rx_callback([this](uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& data)
        {
            this->handle_short_frame(dev_id, cmd, uniq_id, data);            
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

void PcanShortFrameHandler::handle_short_ack(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& data)
{
    AckMessage msg;
    msg.dev_id = dev_id;
    msg.cmd = cmd;
    msg.uniq_id = uniq_id;
    msg.payload = data;
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
                    "[SHORT ACK] dev=%u cmd=0x%08X uniq_id=0x%08X payload_len=%zu",
                    dev_id, static_cast<uint32_t>(cmd), uniq_id, data.size());
    }

    if (cb_copy)
    {
        cb_copy(msg);
    }
}

void PcanShortFrameHandler::handle_short_frame(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& data)
{

    RCLCPP_INFO(rclcpp::get_logger("PcanShortFrameHandler"),
                "[Short RX] dev=%u cmd=0x%08X uniq_id=0x%08X payload_len=%lu",
                dev_id, static_cast<uint32_t>(cmd), uniq_id, data.size());

    switch (cmd)
    {
        case ShortCanCmd::HI:
            handle_short_ack(dev_id, cmd, uniq_id, data);
            break;

        case ShortCanCmd::TIME_SYNC:
            break;

        case ShortCanCmd::HEART_BEAT:
            send_short_time_sync(dev_id, uniq_id);
            break;

        case ShortCanCmd::SENSOR_START:
            break;

        case ShortCanCmd::SENSOR_STOP:
            break;      
              
        case ShortCanCmd::RESET:
            break;

        default:
            RCLCPP_WARN(rclcpp::get_logger("PcanShortFrameHandler"),
                        "[Short RX] Unknown cmd=0x%08X from dev=%u",
                        static_cast<uint32_t>(cmd), dev_id);
            break;
    }

}

bool PcanShortFrameHandler::send_short_time_sync(uint8_t dev_id, uint32_t uniq_id)
{
    struct timespec ts{};
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("PcanShortFrame"),
                     "send_short_time_sync: clock_gettime failed");
        return false;
    }

    /* CLOCK_REALTIME -> Unix epoch 기준 ns 단위 u64로 변환 */
    uint64_t server_ns =
        (static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL) +
        static_cast<uint64_t>(ts.tv_nsec);

    uint8_t payload[8] = {0u, };
    Conversion::u64_to_be(server_ns, payload);

    time_t sec = static_cast<time_t>(ts.tv_sec);
    struct tm tm_info{};
    char time_str[64] = {0, };

    localtime_r(&sec, &tm_info);
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_info);

    if (!quiet_) {
        RCLCPP_INFO(rclcpp::get_logger("PcanShortFrame"),
                    "send_short_time_sync: Time: %s, ns=%llu, dev=%u, uniq_id=0x%08X",
                    time_str,
                    static_cast<unsigned long long>(server_ns),
                    dev_id,
                    uniq_id);
    }

    return can_.send_short_cmd_with_data(dev_id,
                                         ShortCanCmd::TIME_SYNC,
                                         uniq_id,
                                         payload,
                                         sizeof(payload));
}


}  // namespace au_4d_radar