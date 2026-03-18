/**
 * @file pcan_fd_transfer.cpp
 * @author antonioko@au-sensor.com
 * @brief Pure CAN-FD transport layer implementation using the PEAK PCAN USB adapter.
 * @version 1.0
 * @date 2026-03-18
 *
 * @copyright Copyright AU (c) 2026
 */

#include <cstring>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "util/conversion.hpp"

#include "pcan_fd_transfer.hpp"

/**
 * @brief Constructs a PcanFdTransfer instance storing transport and frame configs.
 *
 * @param cfg      PCAN hardware configuration (handle, bitrate, BRS flag).
 * @param short_cfg Short-frame CAN ID base addresses and device count.
 * @param long_cfg  Long-frame CAN ID base addresses, device count and RX buffer size.
 */
PcanFdTransfer::PcanFdTransfer(const Config& cfg,
                               const PcanShortFrameConfig& short_cfg,
                               const PcanLongFrameConfig& long_cfg)
    : cfg_(cfg)
    , short_cfg_(short_cfg)
    , long_cfg_(long_cfg)
{
}

/**
 * @brief Destructor. Shuts down the CAN interface if still initialised.
 */
PcanFdTransfer::~PcanFdTransfer()
{
    shutdown();
}

/**
 * @brief Logs a human-readable PCAN error message.
 *
 * @param tag Prefix string identifying the call site.
 * @param st  PCAN status code to translate.
 */
void PcanFdTransfer::print_pcan_err(const char* tag, TPCANStatus st)
{
    char err[256] = {0};
    CAN_GetErrorText(st, 0, err);
    RCLCPP_ERROR(rclcpp::get_logger("PcanFdTransfer"), "%s: %s (0x%X)", tag, err, static_cast<unsigned>(st));
}

/**
 * @brief Initialises the PCAN FD channel via CAN_InitializeFD().
 *
 * @return true  if the channel was opened successfully (or was already open).
 * @return false if CAN_InitializeFD() returned an error.
 */
bool PcanFdTransfer::init(void)
{
    if (initialized_) {
        return true;
    }

    std::lock_guard<std::mutex> lk(io_mtx_);
    const TPCANStatus st = CAN_InitializeFD(cfg_.handle, const_cast<TPCANBitrateFD>(cfg_.bitrate_fd));
    if (st != PCAN_ERROR_OK) {
        print_pcan_err("CAN_InitializeFD failed", st);
        return false;
    }

    initialized_ = true;

    RCLCPP_DEBUG(
        rclcpp::get_logger("PcanFdTransfer"),
        "PCAN init OK. short(dev=%u tx=0x%03X rx=0x%03X) long(dev=%u tx=0x%03X rx=0x%03X)",
        short_cfg_.device_count,
        short_cfg_.tx_base_id,
        short_cfg_.rx_base_id,
        long_cfg_.device_count,
        long_cfg_.tx_base_id,
        long_cfg_.rx_base_id);

    return true;
}

/**
 * @brief Releases the PCAN FD channel via CAN_Uninitialize().
 *
 * @details Has no effect if the channel was not initialised.
 */
void PcanFdTransfer::shutdown(void)
{
    if (!initialized_) {
        return;
    }

    std::lock_guard<std::mutex> lk(io_mtx_);
    CAN_Uninitialize(cfg_.handle);
    initialized_ = false;
}

/**
 * @brief Converts a payload byte length to the corresponding CAN-FD DLC code.
 *
 * @param len Payload length in bytes (0–64).
 * @return DLC code (0–15) as defined by the CAN-FD standard.
 */
uint8_t PcanFdTransfer::len_to_dlc(uint8_t len)
{
    if (len <= 8u)  return len;
    if (len <= 12u) return 9u;
    if (len <= 16u) return 10u;
    if (len <= 20u) return 11u;
    if (len <= 24u) return 12u;
    if (len <= 32u) return 13u;
    if (len <= 48u) return 14u;
    return 15u;
}

/**
 * @brief Converts a CAN-FD DLC code to the corresponding payload byte length.
 *
 * @param dlc DLC code (0–15).
 * @return Payload length in bytes, or 0 for an out-of-range DLC.
 */
uint8_t PcanFdTransfer::dlc_to_len(uint8_t dlc)
{
    static const uint8_t map[16] = {
        0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
    };

    return (dlc < 16u) ? map[dlc] : 0u;
}

/**
 * @brief Sends a CAN-FD frame with a variable-length payload (up to 64 bytes).
 *
 * @param can_id Standard CAN ID for the outgoing frame.
 * @param data   Pointer to the payload bytes.
 * @param length Payload length in bytes (must be ≤ 64).
 * @return true  on success.
 * @return false if not initialised, data is null, length exceeds 64, or CAN_WriteFD fails.
 */
bool PcanFdTransfer::send_data(uint16_t can_id, const uint8_t* data, uint8_t length)
{
    if (!initialized_ || data == nullptr || length > 64u) {
        return false;
    }

    TPCANMsgFD msg{};
    msg.ID = can_id;
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD | PCAN_MESSAGE_FD;
    if (cfg_.brs_on) {
        msg.MSGTYPE |= PCAN_MESSAGE_BRS;
    }

    msg.DLC = len_to_dlc(length);
    std::memcpy(msg.DATA, data, length);

    std::lock_guard<std::mutex> lk(io_mtx_);
    for (int retry = 0; retry < 3; ++retry) {
        const TPCANStatus st = CAN_WriteFD(cfg_.handle, &msg);
        if (st == PCAN_ERROR_OK) {
            return true;
        }

        print_pcan_err("CAN_WriteFD send_data", st);
        usleep(1000);
    }

    return false;
}

/**
 * @brief Sends a fixed 64-byte CAN-FD frame (DLC=15).
 *
 * @param can_id  Standard CAN ID for the outgoing frame.
 * @param data64  Pointer to exactly 64 bytes of payload.
 * @return true   on success.
 * @return false  if not initialised, data64 is null, or CAN_WriteFD fails.
 */
bool PcanFdTransfer::send_frame64(uint16_t can_id, const uint8_t data64[64])
{
    if (!initialized_ || data64 == nullptr) {
        return false;
    }

    TPCANMsgFD msg{};
    msg.ID = can_id;
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD | PCAN_MESSAGE_FD;
    if (cfg_.brs_on) {
        msg.MSGTYPE |= PCAN_MESSAGE_BRS;
    }

    msg.DLC = 15u;
    std::memcpy(msg.DATA, data64, 64u);

    std::lock_guard<std::mutex> lk(io_mtx_);
    for (int retry = 0; retry < 3; ++retry) {
        const TPCANStatus st = CAN_WriteFD(cfg_.handle, &msg);
        if (st == PCAN_ERROR_OK) {
            return true;
        }

        print_pcan_err("CAN_WriteFD send_frame64", st);
        usleep(1000);
    }

    return false;
}

/**
 * @brief Sends a large application payload using the CustomTP long-frame protocol.
 *
 * @details Internally creates a transient PcanLongFrame instance to split the payload
 *          into 64-byte CAN-FD chunks and transmit them to the target device.
 *
 * @param dev_id      Target device index (0 … device_count-1).
 * @param msg_id      Application message identifier placed in the App-PDU header.
 * @param payload     Pointer to the application payload bytes.
 * @param payload_len Length of the payload in bytes.
 * @return true  if all chunks were transmitted successfully.
 * @return false on any transmission error or invalid arguments.
 */
bool PcanFdTransfer::send_payload(uint8_t dev_id, uint32_t msg_id, const uint8_t* payload, int payload_len)
{
    PcanLongFrame long_frame(*this, long_cfg_);
    return long_frame.send_long_payload(dev_id, msg_id, payload, payload_len);
}

/**
 * @brief Sends a short command frame with an optional data payload.
 *
 * @details Internally creates a transient PcanShortFrame instance to format and
 *          transmit a CMD(4B) + UNIQ_ID(4B) + payload frame to the target device.
 *
 * @param dev_id      Target device index (0 … device_count-1).
 * @param cmd         Short command identifier.
 * @param uniq_id     Unique transaction ID embedded in the frame (big-endian).
 * @param payload     Pointer to optional extra payload bytes (may be nullptr).
 * @param payload_len Length of the extra payload (0–56 bytes).
 * @return true  on success.
 * @return false on any transmission error or invalid arguments.
 */
bool PcanFdTransfer::send_cmd_with_data(uint8_t dev_id,
                                        ShortCanCmd cmd,
                                        uint32_t uniq_id,
                                        const uint8_t* payload,
                                        uint8_t payload_len)
{
    PcanShortFrame short_frame(*this, short_cfg_);
    return short_frame.send_short_command_with_data(dev_id, cmd, uniq_id, payload, payload_len);
}

/**
 * @brief Reads one CAN-FD frame from the hardware receive queue (non-blocking).
 *
 * @param[out] out Populated with the received frame data on success.
 * @return ReadStatus::Ok     if a normal data frame was received.
 * @return ReadStatus::Empty  if the hardware queue contained no frames.
 * @return ReadStatus::Status if a PCAN status/error frame was received.
 * @return ReadStatus::Error  on a hardware or API error.
 */
PcanFdTransfer::ReadStatus PcanFdTransfer::read_frame(RxFrame& out)
{
    out = RxFrame{};

    if (!initialized_) {
        return ReadStatus::Error;
    }

    TPCANMsgFD rx{};
    TPCANTimestampFD ts = 0;

    TPCANStatus st;
    {
        std::lock_guard<std::mutex> lk(io_mtx_);
        st = CAN_ReadFD(cfg_.handle, &rx, &ts);
    }

    if (st == PCAN_ERROR_QRCVEMPTY) {
        return ReadStatus::Empty;
    }

    if (st != PCAN_ERROR_OK) {
        print_pcan_err("CAN_ReadFD", st);
        out.status = st;
        return ReadStatus::Error;
    }

    if ((rx.MSGTYPE & PCAN_MESSAGE_STATUS) != 0u) {
        out.is_status = true;
        out.status = static_cast<TPCANStatus>(Conversion::be_to_u32(rx.DATA));

        print_pcan_err("[STATUS]", out.status);
        return ReadStatus::Status;
    }

    out.can_id = static_cast<uint32_t>(rx.ID);
    out.data_len = dlc_to_len(rx.DLC);
    std::memcpy(out.data, rx.DATA, out.data_len);

    return ReadStatus::Ok;
}
