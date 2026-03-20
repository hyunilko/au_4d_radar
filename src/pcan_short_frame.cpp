/**
 * @file pcan_short_frame.cpp
 * @author antonioko@au-sensor.com
 * @brief Single-frame CAN-FD command/response handler (short-frame protocol).
 * @version 1.0
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 *
 * @details Wire format — Application Layer:
 *          Bytes 00–03: Command ID (4 B, big-endian)
 *          Bytes 04–07: Unique ID   (4 B, big-endian)
 *          Bytes 08+  : Payload     (up to 56 bytes)
 */

#include "pcan_short_frame.hpp"

#include <cstring>
#include <ctime>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "pcan_fd_transfer.hpp"
#include "util/conversion.hpp"

static constexpr uint8_t CMD_FIELD_LEN    = 4u;   /* CMD(4B BE) */
static constexpr uint8_t UNIQ_ID_LEN      = 4u;   /* ACK unique ID */
static constexpr uint8_t SHORT_MAX_BYTES  = 56u;  /* 64B - CMD(4B) - unique ID(4B) */

/**
 * @brief Constructs a PcanShortFrame storing the transport reference and config.
 *
 * @param transport Reference to the transport layer used for sending frames.
 * @param cfg  Short-frame configuration (TX/RX base IDs, device count).
 */
PcanShortFrame::PcanShortFrame(PcanFdTransfer& transport, const Config& cfg)
    : transport_(transport)
    , cfg_(cfg)
{
}

/**
 * @brief Registers the callback invoked when a short frame is received.
 *
 * @param cb Callback: void(dev_id, cmd, uniq_id, payload).
 *           Pass an empty std::function to deregister.
 */
void PcanShortFrame::set_rx_callback(ShortFrameRxCallback cb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    rx_cb_ = std::move(cb);
}

/**
 * @brief Sends a short command frame with no additional payload.
 *
 * @param dev_id  Target device index.
 * @param uniq_id Unique transaction ID.
 * @param cmd     Command identifier.
 * @return true on success, false on error.
 */
bool PcanShortFrame::send_short_command(uint8_t dev_id, uint32_t uniq_id, ShortCanCmd cmd)
{
    return send_short_command_with_data(dev_id, cmd, uniq_id, nullptr, 0u);
}

/**
 * @brief Sends a short command frame with ack.
 *
 * @param dev_id  Target device index.
 * @param uniq_id Unique transaction ID.
 * @param cmd     received Command identifier.
 * @return true on success, false on error.
 */
bool PcanShortFrame::send_short_command_ack(uint8_t dev_id, uint32_t uniq_id, ShortCanCmd rcv_cmd)
{
    uint8_t frame[CMD_FIELD_LEN] = {0u, };
    Conversion::u32_to_be(static_cast<uint32_t>(rcv_cmd), &frame[0]);
    return send_short_command_with_data(dev_id, ShortCanCmd::ACK, uniq_id, frame, CMD_FIELD_LEN);
}

/**
 * @brief Sends a short command frame with an optional extra payload.
 *
 * @details Builds CMD(4B BE) + UNIQ_ID(4B BE) + payload and calls send_data().
 *          Payload is silently truncated to 56 bytes if it exceeds that limit.
 *
 * @param dev_id      Target device index (0 … device_count-1).
 * @param cmd         Command identifier.
 * @param uniq_id     Unique transaction ID.
 * @param payload     Extra payload bytes (nullptr if payload_len == 0).
 * @param payload_len Extra payload length in bytes (max 56).
 * @return true on success, false on error.
 */
bool PcanShortFrame::send_short_command_with_data(uint8_t dev_id,
                                                  ShortCanCmd cmd,
                                                  uint32_t uniq_id,
                                                  const uint8_t* payload,
                                                  uint8_t payload_len)
{
    if (dev_id >= cfg_.device_count) {
        return false;
    }
    if ((payload == nullptr) && (payload_len > 0u)) {
        return false;
    }
    if (payload_len > SHORT_MAX_BYTES) {
        RCLCPP_WARN(rclcpp::get_logger("PcanShortFrame"),
                    "send_short_command_with_data: payload too large (%u > %u), truncating",
                    payload_len, SHORT_MAX_BYTES);
        payload_len = SHORT_MAX_BYTES;
    }

    uint8_t frame[CMD_FIELD_LEN + UNIQ_ID_LEN + SHORT_MAX_BYTES] = {0u, };
    Conversion::u32_to_be(static_cast<uint32_t>(cmd), &frame[0]);
    Conversion::u32_to_be(uniq_id, &frame[4]);

    if ((payload != nullptr) && (payload_len > 0u)) {
        std::memcpy(&frame[CMD_FIELD_LEN + UNIQ_ID_LEN], payload, payload_len);
    }

    const uint16_t can_id = static_cast<uint16_t>(cfg_.tx_base_id + dev_id);
    const uint8_t total_len = static_cast<uint8_t>(CMD_FIELD_LEN + UNIQ_ID_LEN + payload_len);

    const bool ok = transport_.send_data(can_id, frame, total_len);
    if (!ok && !cfg_.quiet) {
        RCLCPP_ERROR(rclcpp::get_logger("PcanShortFrame"),
                     "send_short_command_with_data: send_data failed (dev=%u, cmd=0x%08X)",
                     dev_id, static_cast<uint32_t>(cmd));
    }

    return ok;
}

/**
 * @brief Checks whether a CAN ID belongs to the short-frame RX range.
 *
 * @param can_id      Received CAN ID.
 * @param dev_id_out  Set to the device index when the function returns true.
 * @return true if can_id maps to a valid device, false otherwise.
 */
bool PcanShortFrame::is_short_rx_can_id(uint32_t can_id, uint8_t& dev_id_out) const
{
    if (can_id < cfg_.rx_base_id) {
        return false;
    }

    const uint32_t dev = can_id - cfg_.rx_base_id;
    if (dev >= cfg_.device_count) {
        return false;
    }

    dev_id_out = static_cast<uint8_t>(dev);
    return true;
}

/**
 * @brief Dispatches a CAN frame to the short-frame processor if its ID matches.
 *
 * @param can_id   CAN ID of the received frame.
 * @param data     Frame payload bytes.
 * @param data_len Payload length.
 * @return true if handled, false if the CAN ID is out of range.
 */
bool PcanShortFrame::handle_short_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len)
{
    uint8_t dev_id = 0u;
    if (!is_short_rx_can_id(can_id, dev_id)) {
        return false;
    }

    process_short_frame(dev_id, data, data_len);
    return true;
}

/**
 * @brief Parses a raw short-frame payload and invokes the registered RX callback.
 *
 * @details Extracts CMD(4B BE), UNIQ_ID(4B BE), and optional payload, then calls rx_cb_.
 *
 * @param dev_id   Device index derived from the received CAN ID.
 * @param data     Raw frame payload bytes.
 * @param data_len Payload length (must be ≥ 4 to carry the CMD field).
 */
void PcanShortFrame::process_short_frame(uint8_t dev_id, const uint8_t* data, uint8_t data_len)
{
    if (data == nullptr) {
        return;
    }
    if (data_len < CMD_FIELD_LEN) {
        RCLCPP_WARN(rclcpp::get_logger("PcanShortFrame"),
                    "process_short_frame: frame too short (dev=%u, len=%u)",
                    dev_id, data_len);
        return;
    }

    const uint32_t cmd_raw = Conversion::be_to_u32(&data[0]);
    const auto cmd = static_cast<ShortCanCmd>(cmd_raw);

    uint32_t uniq_id = 0u;
    uint8_t payload_offset = CMD_FIELD_LEN;
    if (data_len >= static_cast<uint8_t>(CMD_FIELD_LEN + UNIQ_ID_LEN)) {
        uniq_id = Conversion::be_to_u32(&data[CMD_FIELD_LEN]);
        payload_offset = static_cast<uint8_t>(CMD_FIELD_LEN + UNIQ_ID_LEN);
    }

    const uint8_t payload_len = (data_len > payload_offset) ? static_cast<uint8_t>(data_len - payload_offset) : 0u;
    const std::vector<uint8_t> payload(data + payload_offset, data + payload_offset + payload_len);

    if (!cfg_.quiet) {
       // RCLCPP_INFO(rclcpp::get_logger("PcanShortFrame"),
       //             "[Short RX] dev=%u cmd=0x%08X uniq_id=0x%08X payload_len=%u",
       //             dev_id, cmd_raw, uniq_id, payload_len);
    }

    std::lock_guard<std::mutex> lk(mtx_);
    if (rx_cb_) {
        rx_cb_(dev_id, cmd, uniq_id, payload);
    }
}

