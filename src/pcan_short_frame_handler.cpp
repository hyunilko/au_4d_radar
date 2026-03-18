/**
 * @file pcan_short_frame_handler.cpp
 * @author antonioko@au-sensor.com
 * @brief High-level handler for short-frame CAN commands (ACK tracking, time-sync).
 * @version 1.1
 * @date 2026-03-18
 *
 * @copyright Copyright AU (c) 2026
 *
 * @details Wraps PcanShortFrame to provide ACK wait/notify semantics, per-device
 *          unique-ID caching, and automatic TIME_SYNC response to HEART_BEAT frames.
 */

#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "util/conversion.hpp"

#include "au_4d_radar.hpp"
#include "pcan_short_frame_handler.hpp"

namespace au_4d_radar
{

/**
 * @brief Constructs a PcanShortFrameHandler.
 *
 * @param node   Pointer to the owning ROS2 radar node (used for publishing and logging).
 * @param can    Reference to the transport layer for sending replies.
 * @param logger ROS2 logger instance; defaults to "PcanShortFrameHandler".
 */
PcanShortFrameHandler::PcanShortFrameHandler(device_au_radar_node* node,
                                             PcanFdTransfer& can,
                                             rclcpp::Logger logger)
    : can_(can)
    , short_frame_(can, can.short_frame_config())
    , logger_(std::move(logger))
    , radar_node_(node)
{
}

/**
 * @brief Installs the short-frame RX callback so incoming frames are processed.
 */
void PcanShortFrameHandler::start(void)
{
    short_frame_.set_rx_callback([this](uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& data)
        {
            this->handle_short_frame(dev_id, cmd, uniq_id, data);
        });


}

/**
 * @brief Deregisters the RX callback and clears all pending ACK state.
 */
void PcanShortFrameHandler::stop(void)
{
    short_frame_.set_rx_callback(PcanShortFrame::ShortFrameRxCallback{});

    std::lock_guard<std::mutex> lk(mtx_);
    ack_cb_ = AckCallback{};
    ack_map_.clear();
    uniq_id_map_.clear();

    RCLCPP_DEBUG(logger_, "PcanShortFrameHandler stopped");
}

/**
 * @brief Dispatches an incoming CAN frame to the owned PcanShortFrame instance.
 *
 * @param can_id   CAN ID of the received frame.
 * @param data     Pointer to frame payload bytes.
 * @param data_len Length of frame payload.
 * @return true  if the frame was consumed by the short-frame handler.
 * @return false if the CAN ID does not belong to the short-frame RX range.
 */
bool PcanShortFrameHandler::handle_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len)
{
    return short_frame_.handle_short_can_frame(can_id, data, data_len);
}

/**
 * @brief Registers a callback that is invoked each time any ACK frame is received.
 *
 * @param cb Callback with signature void(const AckMessage&).
 *           Pass an empty std::function to deregister.
 */
void PcanShortFrameHandler::set_ack_callback(AckCallback cb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    ack_cb_ = std::move(cb);
}

/**
 * @brief Blocks until an ACK is received for the specified (dev_id, cmd) pair or timeout.
 *
 * @param dev_id  Device index to wait for.
 * @param cmd     Command whose ACK is expected.
 * @param out     Populated with the received AckMessage on success.
 * @param timeout Maximum time to wait.
 * @return true  if an ACK was received before the timeout.
 * @return false on timeout.
 */
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
        RCLCPP_WARN(logger_,
                    "wait_for_ack timeout dev=%u cmd=0x%08X timeout_ms=%lld",
                    dev_id, static_cast<uint32_t>(cmd), static_cast<long long>(timeout.count()));
        return false;
    }

    out = ack_map_.at(key);
    return true;
}

/**
 * @brief Retrieves the last known unique ID reported by a device.
 *
 * @param dev_id       Device index to query.
 * @param uniq_id_out  Set to the cached unique ID on success.
 * @return true  if the device has sent at least one non-zero unique ID.
 * @return false if no unique ID has been received from this device yet.
 */
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

/**
 * @brief Builds a 64-bit map key from a device ID and command value.
 *
 * @param dev_id Device index (stored in the upper 32 bits).
 * @param cmd    Short command identifier (stored in the lower 32 bits).
 * @return Combined AckKey for use in the ACK map.
 */
PcanShortFrameHandler::AckKey PcanShortFrameHandler::make_ack_key(uint8_t dev_id, ShortCanCmd cmd)
{
    return (static_cast<AckKey>(dev_id) << 32) | static_cast<uint32_t>(cmd);
}

/**
 * @brief Stores a received ACK frame, caches the device unique ID, and notifies waiters.
 *
 * @param dev_id  Device index that sent the ACK.
 * @param cmd     Command the ACK corresponds to.
 * @param uniq_id Unique ID received in the frame (cached if non-zero).
 * @param data    Optional payload bytes carried in the ACK frame.
 */
void PcanShortFrameHandler::handle_cmd_ack(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& data)
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

    RCLCPP_DEBUG(logger_, "[SHORT ACK] dev=%u cmd=0x%08X uniq_id=0x%08X payload_len=%zu",
                dev_id, static_cast<uint32_t>(cmd), uniq_id, data.size());

    if (cb_copy)
    {
        cb_copy(msg);
    }
}

/**
 * @brief Dispatches a decoded short frame to the appropriate command handler.
 *
 * @param dev_id  Source device index.
 * @param cmd     Decoded command identifier.
 * @param uniq_id Unique ID from the frame header.
 * @param data    Optional payload bytes.
 */
void PcanShortFrameHandler::handle_short_frame(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id, const std::vector<uint8_t>& data)
{

  //  RCLCPP_INFO(logger_,
  //              "[Short RX] dev=%u cmd=0x%08X uniq_id=0x%08X payload_len=%lu",
  //              dev_id, static_cast<uint32_t>(cmd), uniq_id, data.size());

    switch (cmd)
    {
        case ShortCanCmd::HI:
            handle_cmd_ack(dev_id, cmd, uniq_id, data);
            break;

        case ShortCanCmd::TIME_SYNC:
            break;

        case ShortCanCmd::HEART_BEAT:
            send_time_sync(dev_id, uniq_id);
            break;

        case ShortCanCmd::SENSOR_START:
            break;

        case ShortCanCmd::SENSOR_STOP:
            break;

        case ShortCanCmd::RESET:
            break;

        default:
            RCLCPP_WARN(logger_,
                        "[Short RX] Unknown cmd=0x%08X from dev=%u",
                        static_cast<uint32_t>(cmd), dev_id);
            break;
    }

}

/**
 * @brief Sends a TIME_SYNC response containing the current CLOCK_REALTIME timestamp.
 *
 * @details The timestamp is encoded as a big-endian uint64 (nanoseconds since Unix epoch)
 *          and transmitted as an 8-byte payload in a short-frame command.
 *
 * @param dev_id  Target device index.
 * @param uniq_id Unique ID to echo back in the response.
 * @return true on success, false if clock_gettime() fails or the send fails.
 */
bool PcanShortFrameHandler::send_time_sync(uint8_t dev_id, uint32_t uniq_id)
{
    struct timespec ts{};
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        RCLCPP_ERROR(logger_, "send_time_sync: clock_gettime failed");
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

    gmtime_r(&sec, &tm_info);
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_info);

    RCLCPP_DEBUG(radar_node_->get_logger(),
                "send_time_sync: Time: %s, dev=%u, uniq_id=0x%08X",
                time_str, dev_id, Conversion::swap_endian32(uniq_id));

    return can_.send_cmd_with_data(dev_id, ShortCanCmd::TIME_SYNC, uniq_id, payload, sizeof(payload));
}


}  // namespace au_4d_radar