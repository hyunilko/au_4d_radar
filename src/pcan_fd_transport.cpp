/**
 * @file pcan_fd_transport.cpp
 * @author antonioko@au-sensor.com
 * @brief PCAN FD transport layer — hardware I/O, frame routing and callback wrappers.
 * @version 2.0
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 *
 */

#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <sys/epoll.h>

#include "rclcpp/rclcpp.hpp"
#include "util/conversion.hpp"

#include "pcan_fd_transport.hpp"

/* ========================================================================= */
/* Construction / destruction                                                 */
/* ========================================================================= */

/**
 * @brief Constructs a PcanFdTransport and initialises the owned frame objects.
 */
PcanFdTransport::PcanFdTransport(const Config& cfg)
    : cfg_(cfg)
{
    short_frame_ = std::make_unique<PcanShortFrame>(*this, cfg_.short_frame);
    long_frame_  = std::make_unique<PcanLongFrame>(*this,  cfg_.long_frame);
}

/**
 * @brief Destructor — stops the receive thread and releases the PCAN channel.
 */
PcanFdTransport::~PcanFdTransport()
{
    stop_rx();
    shutdown();
}

/* ========================================================================= */
/* Error helpers                                                              */
/* ========================================================================= */

/**
 * @brief Logs a human-readable PCAN error string via RCLCPP_ERROR.
 */
void PcanFdTransport::print_pcan_err(const char* tag, TPCANStatus st)
{
    char err[256] = {0};
    CAN_GetErrorText(st, 0, err);
    RCLCPP_ERROR(rclcpp::get_logger("PcanFdTransport"),
                 "%s: %s (0x%X)", tag, err, static_cast<unsigned>(st));
}

/* ========================================================================= */
/* Channel initialisation / release                                           */
/* ========================================================================= */

/**
 * @brief Opens the PCAN FD channel.
 *
 * [1][2] initialized_ is atomic; the flag is re-read inside io_mtx_ to prevent
 * two concurrent callers from both reaching CAN_InitializeFD().
 *
 * @return true on success or if the channel was already open, false on error.
 */
bool PcanFdTransport::init(void)
{
    /* Fast path — already open.  Atomic load, no lock needed. */
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }

    std::lock_guard<std::mutex> lk(io_mtx_);

    /* [2] Re-check inside the lock to close the TOCTOU window. */
    if (initialized_.load(std::memory_order_relaxed)) {
        return true;
    }

    const TPCANStatus st = CAN_InitializeFD(cfg_.handle,
                         const_cast<TPCANBitrateFD>(cfg_.bitrate_fd));
    if (st != PCAN_ERROR_OK) {
        print_pcan_err("CAN_InitializeFD failed", st);
        return false;
    }

    initialized_.store(true, std::memory_order_release);

    if (!cfg_.long_frame.quiet) {
        RCLCPP_INFO(
            rclcpp::get_logger("PcanFdTransport"),
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
 * @brief Releases the PCAN FD channel. No-op if not initialised.
 */
void PcanFdTransport::shutdown(void)
{
    if (!initialized_.load(std::memory_order_acquire)) {
        return;
    }

    std::lock_guard<std::mutex> lk(io_mtx_);

    /* Re-check inside the lock. */
    if (!initialized_.load(std::memory_order_relaxed)) {
        return;
    }

    CAN_Uninitialize(cfg_.handle);
    initialized_.store(false, std::memory_order_release);
}

/* ========================================================================= */
/* DLC helpers                                                                */
/* ========================================================================= */

/**
 * @brief Maps a payload byte length to the CAN-FD DLC code.
 */
uint8_t PcanFdTransport::len_to_dlc(uint8_t len)
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
 * @brief Maps a CAN-FD DLC code to the payload byte length.
 */
uint8_t PcanFdTransport::dlc_to_len(uint8_t dlc)
{
    static const uint8_t map[16] = {
        0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
    };
    return (dlc < 16u) ? map[dlc] : 0u;
}

/* ========================================================================= */
/* [3][4] Shared write path                                                   */
/* ========================================================================= */

/**
 * @brief Attempts to write @p msg to the PCAN channel with up to TX_MAX_RETRIES.
 *
 * @param msg  Fully populated TPCANMsgFD ready for CAN_WriteFD().
 * @return true on success, false if all retries are exhausted.
 */
bool PcanFdTransport::do_can_write(TPCANMsgFD& msg)
{
    for (int retry = 0; retry < TX_MAX_RETRIES; ++retry) {
        TPCANStatus st;
        {
            std::lock_guard<std::mutex> lk(io_mtx_);
            st = CAN_WriteFD(cfg_.handle, &msg);
        }

        if (st == PCAN_ERROR_OK) {
            return true;
        }

        if (!cfg_.long_frame.quiet) {
            print_pcan_err("CAN_WriteFD", st);
        }

        /* [4] Sleep without the lock so poll_rx() is not blocked. */
        usleep(static_cast<unsigned>(TX_RETRY_DELAY_US));
    }

    return false;
}

/* ========================================================================= */
/* Public send API                                                            */
/* ========================================================================= */

/**
 * @brief Sends a CAN-FD frame with a variable-length payload (up to 64 bytes).
 */
bool PcanFdTransport::send_data(uint16_t can_id, const uint8_t* data, uint8_t length)
{
    if (!initialized_.load(std::memory_order_acquire) ||
        data   == nullptr                             ||
        length  > 64u)
    {
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

    return do_can_write(msg);
}

/**
 * @brief Sends a fixed 64-byte CAN-FD frame (DLC = 15).
 */
bool PcanFdTransport::send_frame64(uint16_t can_id, const uint8_t data64[64])
{
    if (!initialized_.load(std::memory_order_acquire) || data64 == nullptr) {
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

    return do_can_write(msg);
}

/* ========================================================================= */
/* Frame dispatch                                                             */
/* ========================================================================= */

/**
 * @brief Drains the hardware RX queue and dispatches each frame to the handler.
 *
 * Calls CAN_ReadFD() in a loop until PCAN_ERROR_QRCVEMPTY.
 * Each data frame is offered to short_frame_ first, then long_frame_.
 */
void PcanFdTransport::poll_rx(void)
{
    if (!initialized_.load(std::memory_order_acquire)) {
        return;
    }

    while (true) {
        TPCANMsgFD       rx{};
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

        if (short_frame_ && short_frame_->handle_ack_can_frame(can_id, rx.DATA, data_len)) {
            continue;
        }

        if (!cfg_.long_frame.quiet) {
            RCLCPP_WARN(rclcpp::get_logger("PcanFdTransport"),
                        "Unknown CAN ID: 0x%03X len=%u", can_id, data_len);
        }
    }
}

/* ========================================================================= */
/* [5] epoll setup                                                            */
/* ========================================================================= */

/**
 * @brief Registers the PCAN receive-event fd with a epoll instance.
 *
 * On Linux, CAN_GetValue(PCAN_RECEIVE_EVENT) returns a file descriptor that
 * becomes readable whenever a frame arrives in the hardware FIFO.  Registering
 * it with epoll lets receiveThread() block until work actually arrives instead
 * of spinning at 1 kHz.
 *
 * @return epoll fd (≥ 0) on success, -1 if setup is not supported.
 */
int PcanFdTransport::setup_epoll(void)
{
    int event_fd = -1;
    const TPCANStatus st =
        CAN_GetValue(cfg_.handle,
                     PCAN_RECEIVE_EVENT,
                     &event_fd,
                     sizeof(event_fd));

    if (st != PCAN_ERROR_OK || event_fd < 0) {
        if (!cfg_.long_frame.quiet) {
            RCLCPP_WARN(rclcpp::get_logger("PcanFdTransport"),
                        "PCAN_RECEIVE_EVENT not available (0x%X)"
                        , static_cast<unsigned>(st));
        }
        return -1;
    }

    const int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        RCLCPP_WARN(rclcpp::get_logger("PcanFdTransport"),
                    "epoll_create1 failed (%s)",
                    strerror(errno));
        return -1;
    }

    epoll_event ev{};
    ev.events  = EPOLLIN;
    ev.data.fd = event_fd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, event_fd, &ev) < 0) {
        RCLCPP_WARN(rclcpp::get_logger("PcanFdTransport"),
                    "epoll_ctl failed (%s)",
                    strerror(errno));
        close(epfd);
        return -1;
    }

    rx_event_fd_ = event_fd;
    return epfd;
}

/* ========================================================================= */
/* Receive loop                                                               */
/* ========================================================================= */

/**
 * @brief Convenience method — initialises the channel then starts the receive thread.
 */
void PcanFdTransport::start(void)
{
    init();
    start_rx();
}

/**
 * @brief Starts the CAN FD receive thread. No-op if already running.
 */
void PcanFdTransport::start_rx(void)
{
    if (rx_running_.load()) {
        return;
    }
    rx_running_.store(true);
    rx_thread_ = std::thread(&PcanFdTransport::receiveThread, this);
}

/**
 * @brief Signals the receive thread to stop and joins it.
 */
void PcanFdTransport::stop_rx(void)
{
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }

    /* Clean up epoll resources. */
    if (rx_epoll_fd_ >= 0) {
        close(rx_epoll_fd_);
        rx_epoll_fd_ = -1;
    }
    rx_event_fd_ = -1;
}

/**
 * @brief Receive loop body.
 *
 * The epoll timeout (EPOLL_TIMEOUT_MS) ensures rx_running_ is checked
 * periodically even when no frames arrive, so stop_rx() always terminates
 * the thread promptly.
 */
void PcanFdTransport::receiveThread(void)
{
    /* [5] Try to set up event-driven receive. */
    rx_epoll_fd_ = setup_epoll();
    bool use_epoll = (rx_epoll_fd_ >= 0);

    if (!cfg_.long_frame.quiet) {
        RCLCPP_INFO(rclcpp::get_logger("PcanFdTransport"),
                    "Receive thread started — mode: %s",
                    use_epoll ? "epoll" : "usleep polling");
    }

    epoll_event events[8];

    while (rx_running_.load()) {
        if (use_epoll) {
            /* Block until a frame arrives or the timeout expires. */
            const int n = epoll_wait(rx_epoll_fd_, events, 8, EPOLL_TIMEOUT_MS);

            if (n < 0) {
                if (errno == EINTR) {
                    continue;  /* Interrupted by a signal — retry. */
                }
                RCLCPP_ERROR(rclcpp::get_logger("PcanFdTransport"),
                             "epoll_wait error: %s", strerror(errno));
                use_epoll = false;
                continue;
            }

            /* n == 0: timeout — loop back to check rx_running_. */
            if (n > 0) {
                poll_rx();
            }
        } else {
            poll_rx();
            usleep(1000);
        }
    }

    if (!cfg_.long_frame.quiet) {
        RCLCPP_INFO(rclcpp::get_logger("PcanFdTransport"), "Receive thread stopped.");
    }
}

/* ========================================================================= */
/* Protocol layer accessors                                                   */
/* ========================================================================= */

/**
 * @brief Returns a reference to the owned PcanLongFrame instance.
 */
PcanLongFrame& PcanFdTransport::long_frame()
{
    return *long_frame_;
}

/**
 * @brief Returns a reference to the owned PcanShortFrame instance.
 */
PcanShortFrame& PcanFdTransport::short_frame()
{
    return *short_frame_;
}
