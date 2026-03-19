/**
 * @file pcan_short_frame_handler.cpp
 * @author antonioko@au-sensor.com
 * @brief High-level short-frame handler: ACK tracking, unique-ID cache, and time-sync.
 * @version 1.0
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 *
 * @details Wraps PcanFdTransfer's short-frame callback to provide:
 *          - Per-device unique-ID caching (get_device_uniq_id).
 *          - Automatic TIME_SYNC response whenever a HEART_BEAT is received.
 */

#include <ctime>
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
 * @param node   Pointer to the owning radar node (for publishing and logging).
 * @param can    Reference to the transport layer.
 * @param logger ROS2 logger; defaults to "PcanShortFrameHandler".
 * @param quiet  If true, suppresses INFO/DEBUG log output.
 */
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

/**
 * @brief Installs the short-frame RX callback so incoming frames are dispatched.
 */
void PcanShortFrameHandler::start(void)
{
    can_.set_short_rx_callback(
        [this](uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id,
               const std::vector<uint8_t>& data)
        {
            handle_short_frame(dev_id, cmd, uniq_id, data);
        });
}

/**
 * @brief Deregisters the RX callback and clears all pending ACK state.
 */
void PcanShortFrameHandler::stop(void)
{
    can_.set_short_rx_callback(PcanFdTransfer::ShortRxCallback{});

    std::lock_guard<std::mutex> lk(mtx_);
    ack_cb_ = AckCallback{};
    ack_map_.clear();
    uniq_id_map_.clear();

    if (!quiet_) {
        RCLCPP_INFO(logger_, "PcanShortFrameHandler stopped");
    }
}

/**
 * @brief Registers a callback notified each time any ACK frame is received.
 *
 * @param cb Callback: void(const AckMessage&). Pass empty to deregister.
 */
void PcanShortFrameHandler::set_ack_callback(AckCallback cb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    ack_cb_ = std::move(cb);
}

/**
 * @brief Blocks until an ACK for (dev_id, cmd) is received or the timeout expires.
 *
 * @param dev_id  Device index to wait for.
 * @param cmd     Command whose ACK is expected.
 * @param out     Populated with the received AckMessage on success.
 * @param timeout Maximum wait duration.
 * @return true if ACK received, false on timeout.
 */
bool PcanShortFrameHandler::wait_for_ack(uint8_t dev_id,
                                          ShortCanCmd cmd,
                                          AckMessage& out,
                                          std::chrono::milliseconds timeout)
{
    const AckKey key = make_ack_key(dev_id, cmd);

    std::unique_lock<std::mutex> lk(mtx_);
    const bool ok = cv_.wait_for(lk, timeout, [this, key] {
        return ack_map_.find(key) != ack_map_.end();
    });

    if (!ok) {
        if (!quiet_) {
            RCLCPP_WARN(logger_,
                        "wait_for_ack timeout dev=%u cmd=0x%08X timeout_ms=%lld",
                        dev_id, static_cast<uint32_t>(cmd),
                        static_cast<long long>(timeout.count()));
        }
        return false;
    }

    out = ack_map_.at(key);
    return true;
}

/**
 * @brief Returns the last known unique ID reported by a device.
 *
 * @param dev_id       Device index to query.
 * @param uniq_id_out  Set to the cached unique ID on success.
 * @return true if at least one non-zero unique ID has been received, false otherwise.
 */
bool PcanShortFrameHandler::get_device_uniq_id(uint8_t dev_id,
                                                uint32_t& uniq_id_out) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    const auto it = uniq_id_map_.find(dev_id);
    if (it == uniq_id_map_.end()) {
        return false;
    }
    uniq_id_out = it->second;
    return true;
}

/**
 * @brief Builds a 64-bit ACK map key from a device index and command value.
 *
 * @param dev_id Device index (stored in upper 32 bits).
 * @param cmd    Command identifier (stored in lower 32 bits).
 * @return Combined AckKey.
 */
PcanShortFrameHandler::AckKey
PcanShortFrameHandler::make_ack_key(uint8_t dev_id, ShortCanCmd cmd)
{
    return (static_cast<AckKey>(dev_id) << 32) |
            static_cast<uint32_t>(cmd);
}

/**
 * @brief Stores a received ACK, caches the device unique ID, and wakes any waiters.
 *
 * @param dev_id  Source device index.
 * @param cmd     Command the ACK corresponds to.
 * @param uniq_id Unique ID in the frame (cached if non-zero).
 * @param data    Optional payload carried in the ACK.
 */
void PcanShortFrameHandler::handle_cmd_ack(uint8_t dev_id, ShortCanCmd cmd,
                                            uint32_t uniq_id,
                                            const std::vector<uint8_t>& data)
{
    AckMessage msg;
    msg.dev_id  = dev_id;
    msg.cmd     = cmd;
    msg.uniq_id = uniq_id;
    msg.payload = data;
    msg.rx_time = std::chrono::steady_clock::now();

    AckCallback cb_copy;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        ack_map_[make_ack_key(dev_id, cmd)] = msg;
        if (uniq_id != 0u) {
            uniq_id_map_[dev_id] = uniq_id;
        }
        cb_copy = ack_cb_;
    }

    cv_.notify_all();

    if (!quiet_) {
        RCLCPP_INFO(logger_,
                    "[SHORT ACK] dev=%u cmd=0x%08X uniq_id=0x%08X payload_len=%zu",
                    dev_id, static_cast<uint32_t>(cmd), uniq_id, data.size());
    }

    if (cb_copy) {
        cb_copy(msg);
    }
}

/**
 * @brief Dispatches a decoded short frame to the appropriate command handler.
 *
 * @param dev_id  Source device index.
 * @param cmd     Decoded command identifier.
 * @param uniq_id Unique ID from the frame.
 * @param data    Optional payload bytes.
 */
void PcanShortFrameHandler::handle_short_frame(uint8_t dev_id, ShortCanCmd cmd,
                                                uint32_t uniq_id,
                                                const std::vector<uint8_t>& data)
{
    switch (cmd) {
        case ShortCanCmd::HI:
            handle_cmd_ack(dev_id, cmd, uniq_id, data);
            break;

        case ShortCanCmd::HEART_BEAT:
            send_time_sync(dev_id, uniq_id);
            break;

        case ShortCanCmd::TIME_SYNC:
        case ShortCanCmd::SENSOR_START:
        case ShortCanCmd::SENSOR_STOP:
        case ShortCanCmd::RESET:
            /* ACK frames — no action required */
            break;

        default:
            RCLCPP_WARN(logger_,
                        "[Short RX] Unknown cmd=0x%08X from dev=%u",
                        static_cast<uint32_t>(cmd), dev_id);
            break;
    }
}

/**
 * @brief Sends a TIME_SYNC response with the current CLOCK_REALTIME timestamp.
 *
 * @details Encodes the time as a big-endian uint64 (nanoseconds since Unix epoch)
 *          and transmits it as an 8-byte short-frame payload.
 *
 * @param dev_id  Target device index.
 * @param uniq_id Unique ID to echo back.
 * @return true on success, false if clock_gettime() or the send fails.
 */
bool PcanShortFrameHandler::send_time_sync(uint8_t dev_id, uint32_t uniq_id)
{
    struct timespec ts{};
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        RCLCPP_ERROR(logger_, "send_time_sync: clock_gettime failed");
        return false;
    }

    const uint64_t server_ns =
        (static_cast<uint64_t>(ts.tv_sec)  * 1'000'000'000ULL) +
         static_cast<uint64_t>(ts.tv_nsec);

    uint8_t payload[8] = {0u};
    Conversion::u64_to_be(server_ns, payload);

    if (!quiet_) {
        char     time_str[64] = {0};
        struct tm tm_info{};
        const time_t sec = static_cast<time_t>(ts.tv_sec);
        gmtime_r(&sec, &tm_info);
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_info);
        RCLCPP_DEBUG(radar_node_->get_logger(),
                     "send_time_sync: %s  dev=%u uniq_id=0x%08X",
                     time_str, dev_id, Conversion::swap_endian32(uniq_id));
    }

    return can_.short_frame().send_short_command_with_data(
               dev_id, ShortCanCmd::TIME_SYNC, uniq_id, payload, sizeof(payload));
}

} // namespace au_4d_radar
