/**
 * @file pcan_short_frame_handler.cpp
 * @author antonioko@au-sensor.com
 * @brief High-level short-frame handler: time-sync. Business logic
 * @version 1.0
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 *
 * @details Wraps PcanFdTransport's short-frame callback to provide:
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
 * @param can    Reference to PcanShortFrame (direct, no PcanFdTransport indirection).
 * @param logger ROS2 logger; defaults to "PcanShortFrameHandler".
 * @param quiet  If true, suppresses INFO/DEBUG log output.
 */
PcanShortFrameHandler::PcanShortFrameHandler(device_au_radar_node* node,
                                             PcanShortFrame& can,
                                             rclcpp::Logger logger,
                                             bool quiet)
    : can_short_(can)
    , logger_(std::move(logger))
    , quiet_(quiet)
    , radar_node_(node)
{
}

/**
 * @brief Registers the RX callback on PcanShortFrame and forwards received short frames
 *        to handle_short_frame().
 */
void PcanShortFrameHandler::start(void)
{
    can_short_.set_rx_callback(
        [this](uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id,
               const std::vector<uint8_t>& data)
        {
            handle_short_frame(dev_id, cmd, uniq_id, data);
        });
}

/**
 * @brief Deregisters the PcanShortFrame RX callback.
 */
void PcanShortFrameHandler::stop(void)
{
    can_short_.set_rx_callback(PcanShortFrame::ShortFrameRxCallback{});

    if (!quiet_) {
        RCLCPP_INFO(logger_, "PcanShortFrameHandler stopped");
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
    (void)data;

    switch (cmd) {
        case ShortCanCmd::HI:
            can_short_.send_short_command_ack(dev_id, uniq_id, cmd);
            break;

        case ShortCanCmd::HEART_BEAT:
            send_time_sync(dev_id, uniq_id);
            break;

        case ShortCanCmd::TIME_SYNC:
        case ShortCanCmd::SENSOR_START:
        case ShortCanCmd::SENSOR_STOP:
        case ShortCanCmd::RESET:
        case ShortCanCmd::ACK:
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

    return can_short_.send_short_command_with_data(
               dev_id, ShortCanCmd::TIME_SYNC, uniq_id, payload, sizeof(payload));
}

} // namespace au_4d_radar
