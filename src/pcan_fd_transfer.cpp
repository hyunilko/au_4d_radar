/**
 * @file pcan_fd_transfer.cpp
 * @author antonioko@au-sensor.com
 * @brief PCAN FD transport layer — hardware I/O, frame routing and callback wrappers.
 * @version 1.0
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 *
 * @details Owns the PCAN FD channel and the PcanShortFrame / PcanLongFrame objects.
 *          Exposes poll_rx() for frame-driven dispatch, plus send helpers and
 *          callback registration wrappers forwarded to the owned sub-objects.
 */

#include <cstring>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "util/conversion.hpp"

#include "pcan_fd_transfer.hpp"

/**
 * @brief Constructs a PcanFdTransfer and initialises the owned frame objects.
 *
 * @details Creates PcanShortFrame and PcanLongFrame instances using the
 *          short_frame and long_frame sub-configs embedded in @p cfg.
 *
 * @param cfg Unified configuration holding PCAN hardware settings and both
 *            short-frame and long-frame sub-configs.
 */
PcanFdTransfer::PcanFdTransfer(const Config& cfg)
    : cfg_(cfg)
{
    short_frame_ = std::make_unique<PcanShortFrame>(*this, cfg_.short_frame);
    long_frame_  = std::make_unique<PcanLongFrame>(*this,  cfg_.long_frame);
}

/**
 * @brief Destructor. Calls stop_rx() then shutdown() to release the PCAN channel.
 */
PcanFdTransfer::~PcanFdTransfer()
{
    stop_rx();
    shutdown();
}

/**
 * @brief Logs a human-readable PCAN error string via RCLCPP_ERROR.
 *
 * @param tag Prefix label identifying the call site.
 * @param st  PCAN status code to translate via CAN_GetErrorText().
 */
void PcanFdTransfer::print_pcan_err(const char* tag, TPCANStatus st)
{
    char err[256] = {0};
    CAN_GetErrorText(st, 0, err);
    RCLCPP_ERROR(rclcpp::get_logger("PcanFdTransfer"),
                 "%s: %s (0x%X)", tag, err, static_cast<unsigned>(st));
}

/**
 * @brief Opens the PCAN FD channel via CAN_InitializeFD().
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
    const TPCANStatus st =
        CAN_InitializeFD(cfg_.handle, const_cast<TPCANBitrateFD>(cfg_.bitrate_fd));
    if (st != PCAN_ERROR_OK) {
        print_pcan_err("CAN_InitializeFD failed", st);
        return false;
    }

    initialized_ = true;

    if (!cfg_.long_frame.quiet) {
        RCLCPP_INFO(
            rclcpp::get_logger("PcanFdTransfer"),
            "PCAN init OK. dev=%u short_tx=0x%03X short_rx=0x%03X "
            "long_tx=0x%03X long_rx=0x%03X",
            cfg_.long_frame.device_count,
            cfg_.short_frame.tx_base_id,
            cfg_.short_frame.rx_base_id,
            cfg_.long_frame.tx_base_id,
            cfg_.long_frame.rx_base_id);
    }

    return true;
}

/**
 * @brief Releases the PCAN FD channel via CAN_Uninitialize(). No-op if not initialised.
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
 * @brief Maps a payload byte length to the corresponding CAN-FD DLC code.
 *
 * @param len Payload length in bytes (0–64).
 * @return DLC code (0–15).
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
 * @brief Maps a CAN-FD DLC code to the corresponding payload byte length.
 *
 * @param dlc DLC code (0–15).
 * @return Payload length in bytes, or 0 for an invalid DLC.
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
 * @param can_id Standard CAN ID.
 * @param data   Payload bytes.
 * @param length Payload length (must be ≤ 64).
 * @return true on success, false on invalid arguments or send failure.
 */
bool PcanFdTransfer::send_data(uint16_t can_id, const uint8_t* data, uint8_t length)
{
    if (!initialized_ || data == nullptr || length > 64u) {
        return false;
    }

    TPCANMsgFD msg{};
    msg.ID      = can_id;
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
        if (!cfg_.long_frame.quiet) {
            print_pcan_err("CAN_WriteFD send_data", st);
        }
        usleep(1000);
    }
    return false;
}

/**
 * @brief Sends a fixed 64-byte CAN-FD frame (DLC = 15).
 *
 * @param can_id  Standard CAN ID.
 * @param data64  Pointer to exactly 64 bytes of payload.
 * @return true on success, false on invalid arguments or send failure.
 */
bool PcanFdTransfer::send_frame64(uint16_t can_id, const uint8_t data64[64])
{
    if (!initialized_ || data64 == nullptr) {
        return false;
    }

    TPCANMsgFD msg{};
    msg.ID      = can_id;
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
        if (!cfg_.long_frame.quiet) {
            print_pcan_err("CAN_WriteFD send_frame64", st);
        }
        usleep(1000);
    }
    return false;
}

/**
 * @brief Drains the hardware RX queue and dispatches each frame to the appropriate handler.
 *
 * @details Calls CAN_ReadFD() in a loop until the queue is empty (PCAN_ERROR_QRCVEMPTY).
 *          Each data frame is offered to short_frame_ first, then to long_frame_.
 *          Status frames and unrecognised CAN IDs are logged when quiet mode is off.
 */
void PcanFdTransfer::poll_rx(void)
{
    if (!initialized_) {
        return;
    }

    while (true) {
        TPCANMsgFD    rx{};
        TPCANTimestampFD ts = 0;

        TPCANStatus st;
        {
            std::lock_guard<std::mutex> lk(io_mtx_);
            st = CAN_ReadFD(cfg_.handle, &rx, &ts);
        }

        if (st == PCAN_ERROR_QRCVEMPTY) {
            break;
        }

        if (st != PCAN_ERROR_OK) {
            if (!cfg_.long_frame.quiet) {
                print_pcan_err("CAN_ReadFD", st);
            }
            continue;
        }

        if ((rx.MSGTYPE & PCAN_MESSAGE_STATUS) != 0u) {
            if (!cfg_.long_frame.quiet) {
                const uint32_t ec = Conversion::be_to_u32(rx.DATA);
                print_pcan_err("[STATUS]", static_cast<TPCANStatus>(ec));
            }
            continue;
        }

        const uint8_t  data_len = dlc_to_len(rx.DLC);
        const uint32_t can_id   = static_cast<uint32_t>(rx.ID);

        if (short_frame_ && short_frame_->handle_short_can_frame(can_id, rx.DATA, data_len)) {
            continue;
        }

        if (long_frame_ && long_frame_->handle_long_can_frame(can_id, rx.DATA, data_len)) {
            continue;
        }

        if (!cfg_.long_frame.quiet) {
            RCLCPP_WARN(rclcpp::get_logger("PcanFdTransfer"),
                        "Unknown CAN ID: 0x%03X len=%u", can_id, data_len);
        }
    }
}

/* ---------- Receive loop (owned by the transport layer) ------------------ */

/**
 * @brief Convenience method that initialises the channel and starts the receive thread.
 *
 * @details Call this after all Long / Short handler callbacks have been registered
 *          in au_4d_radar.cpp.  Internally guarantees the order init() → start_rx().
 */
void PcanFdTransfer::start(void)
{
    init();
    start_rx();
}

/**
 * @brief Starts the CAN FD receive thread.
 *
 * @details Must be called after handler RX callbacks have been registered on
 *          PcanLongFrame / PcanShortFrame, otherwise frames will not be delivered.
 *          If the thread is already running this is a no-op.
 */
void PcanFdTransfer::start_rx(void)
{
    if (rx_running_.load()) {
        return;
    }
    rx_running_.store(true);
    rx_thread_ = std::thread(&PcanFdTransfer::receiveThread, this);
}

/**
 * @brief Signals the receive thread to stop and joins it.
 */
void PcanFdTransfer::stop_rx(void)
{
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
}

/**
 * @brief Receive loop body — calls poll_rx() repeatedly at 1 ms intervals.
 *
 * @details Both Short and Long frames are processed on this thread, so frames
 *          are delivered independently to both PcanLongFrameHandler and
 *          PcanShortFrameHandler.
 */
void PcanFdTransfer::receiveThread(void)
{
    while (rx_running_.load()) {
        poll_rx();
        usleep(1000);
    }
}

/* ---------- Protocol layer accessors ------------------------------------- */

/**
 * @brief Returns a reference to the owned PcanLongFrame instance.
 *
 * @details Handlers call send_long_payload() directly on the returned object
 *          instead of going through the now-removed PcanFdTransfer::send_payload()
 *          wrapper.  Asserts that long_frame_ has been constructed.
 */
PcanLongFrame& PcanFdTransfer::long_frame()
{
    return *long_frame_;
}

/**
 * @brief Returns a reference to the owned PcanShortFrame instance.
 *
 * @details Handlers call send_short_command_with_data() directly on the returned
 *          object instead of going through the now-removed
 *          PcanFdTransfer::send_cmd_with_data() wrapper.
 *          Asserts that short_frame_ has been constructed.
 */
PcanShortFrame& PcanFdTransfer::short_frame()
{
    return *short_frame_;
}
