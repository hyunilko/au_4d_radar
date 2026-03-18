/**
 * @file pcan_short_frame.cpp
 * @author antonioko@au-sensor.com
 * @brief Single-frame CAN-FD command/response handler for the short-frame protocol.
 * @version 1.1
 * @date 2026-03-18
 *
 * @copyright Copyright AU (c) 2026
 *
 * @details Implements the short-frame wire format:
 *          CMD(4B BE) + UNIQ_ID(4B BE) + optional payload (up to 56 bytes).
 *          Frames are sent and received within a single 64-byte CAN-FD message.
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
 * @brief Constructs a PcanShortFrame instance.
 *
 * @param pcan Reference to the underlying transport layer used for sending frames.
 * @param cfg  Short-frame configuration (TX/RX base CAN IDs and device count).
 */
PcanShortFrame::PcanShortFrame(PcanFdTransfer& pcan, const Config& cfg)
    : pcan_(pcan)
    , cfg_(cfg)
{
}

/**
 * @brief Registers the callback invoked when a short frame is received.
 *
 * @param cb Callback with signature
 *           void(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id,
 *                const std::vector<uint8_t>& payload).
 *           Pass an empty std::function to deregister.
 */
void PcanShortFrame::set_rx_callback(ShortFrameRxCallback cb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    rx_cb_ = std::move(cb);
}

/**
 * @brief Sends a short command frame without any additional payload.
 *
 * @param dev_id  Target device index (0 … device_count-1).
 * @param uniq_id Unique transaction ID (big-endian in wire format).
 * @param cmd     Command identifier to send.
 * @return true on success, false on error.
 */
bool PcanShortFrame::send_short_command(uint8_t dev_id, uint32_t uniq_id, ShortCanCmd cmd)
{
    return send_short_command_with_data(dev_id, cmd, uniq_id, nullptr, 0u);
}

/**
 * @brief Sends a short command frame with an optional extra payload.
 *
 * @details Builds a frame: CMD(4B BE) + UNIQ_ID(4B BE) + payload, then calls
 *          PcanFdTransfer::send_data(). Payload is silently truncated to 56 bytes
 *          if it exceeds that limit.
 *
 * @param dev_id      Target device index (0 … device_count-1).
 * @param cmd         Command identifier.
 * @param uniq_id     Unique transaction ID.
 * @param payload     Pointer to extra payload bytes (may be nullptr if payload_len == 0).
 * @param payload_len Length of extra payload in bytes (max 56).
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

    const bool ok = pcan_.send_data(can_id, frame, total_len);
    if (!ok) {
        RCLCPP_ERROR(rclcpp::get_logger("PcanShortFrame"),
                     "send_short_command_with_data: send_data failed (dev=%u, cmd=0x%08X)",
                     dev_id, static_cast<uint32_t>(cmd));
    }

    return ok;
}

/**
 * @brief Checks whether a CAN ID falls in the configured short-frame RX range.
 *
 * @param can_id      Received CAN ID to test.
 * @param dev_id_out  Set to the computed device index when the function returns true.
 * @return true  if can_id maps to a valid device index.
 * @return false otherwise.
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
 * @brief Dispatches an incoming CAN frame to the short-frame processor if its ID matches.
 *
 * @param can_id   CAN ID of the received frame.
 * @param data     Pointer to the frame payload bytes.
 * @param data_len Length of the frame payload.
 * @return true  if the CAN ID was handled by this instance.
 * @return false if the CAN ID does not belong to the short-frame RX range.
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
 * @details Wire layout (Application Layer):
 *          - Bytes 00–03: Command ID (4 bytes, big-endian)
 *          - Bytes 04–07: Unique ID   (4 bytes, big-endian)
 *          - Bytes 08+  : Payload     (variable, up to 56 bytes)
 *
 * @param dev_id   Device index derived from the received CAN ID.
 * @param data     Pointer to the raw frame payload bytes.
 * @param data_len Payload length in bytes (must be ≥ 4 to carry at least the CMD field).
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


    std::lock_guard<std::mutex> lk(mtx_);
    if (rx_cb_) {
        rx_cb_(dev_id, cmd, uniq_id, payload);
    }
}

