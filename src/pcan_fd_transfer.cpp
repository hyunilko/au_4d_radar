#include "pcan_fd_transfer.hpp"

#include <cstring>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "util/conversion.hpp"

PcanFdTransfer::PcanFdTransfer(const Config& cfg)
    : cfg_(cfg)
{
    short_frame_ = std::make_unique<PcanShortFrame>(
        *this,
        PcanShortFrame::Config{cfg_.short_tx_base_id, cfg_.short_rx_base_id, cfg_.device_count, cfg_.quiet});

    long_frame_ = std::make_unique<PcanLongFrame>(
        *this,
        PcanLongFrame::Config{cfg_.long_tx_base_id,
                              cfg_.long_rx_base_id,
                              cfg_.device_count,
                              cfg_.rx_buf_size,
                              cfg_.quiet});
}

PcanFdTransfer::~PcanFdTransfer()
{
    shutdown();
}

void PcanFdTransfer::print_pcan_err(const char* tag, TPCANStatus st)
{
    char err[256] = {0};
    CAN_GetErrorText(st, 0, err);
    RCLCPP_ERROR(rclcpp::get_logger("PcanFdTransfer"), "%s: %s (0x%X)", tag, err, static_cast<unsigned>(st));
}

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

    if (!cfg_.quiet) {
        RCLCPP_INFO(
            rclcpp::get_logger("PcanFdTransfer"),
            "PCAN init OK. dev=%u short_tx=0x%03X short_rx=0x%03X long_tx=0x%03X long_rx=0x%03X",
            cfg_.device_count,
            cfg_.short_tx_base_id,
            cfg_.short_rx_base_id,
            cfg_.long_tx_base_id,
            cfg_.long_rx_base_id);
    }

    return true;
}

void PcanFdTransfer::shutdown(void)
{
    if (!initialized_) {
        return;
    }

    std::lock_guard<std::mutex> lk(io_mtx_);
    CAN_Uninitialize(cfg_.handle);
    initialized_ = false;
}

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

uint8_t PcanFdTransfer::dlc_to_len(uint8_t dlc)
{
    static const uint8_t map[16] = {
        0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
    };

    return (dlc < 16u) ? map[dlc] : 0u;
}

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

        if (!cfg_.quiet) {
            print_pcan_err("CAN_WriteFD send_data", st);
        }
        usleep(1000);
    }

    return false;
}

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

        if (!cfg_.quiet) {
            print_pcan_err("CAN_WriteFD send_frame64", st);
        }
        usleep(1000);
    }

    return false;
}

void PcanFdTransfer::poll_rx(void)
{
    if (!initialized_) {
        return;
    }

    while (true) {
        TPCANMsgFD rx{};
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
            if (!cfg_.quiet) {
                print_pcan_err("CAN_ReadFD", st);
            }
            continue;
        }

        if ((rx.MSGTYPE & PCAN_MESSAGE_STATUS) != 0u) {
            if (!cfg_.quiet) {
                const uint32_t ec = Conversion::be_to_u32(rx.DATA);
                print_pcan_err("[STATUS]", static_cast<TPCANStatus>(ec));
            }
            continue;
        }

        const uint8_t data_len = dlc_to_len(rx.DLC);
        const uint32_t can_id = static_cast<uint32_t>(rx.ID);

        if (short_frame_ && short_frame_->handle_short_can_frame(can_id, rx.DATA, data_len)) {
            continue;
        }

        if (long_frame_ && long_frame_->handle_long_can_frame(can_id, rx.DATA, data_len)) {
            continue;
        }

        if (!cfg_.quiet) {
            RCLCPP_WARN(rclcpp::get_logger("PcanFdTransfer"),
                        "Unknown CAN ID: 0x%03X len=%u",
                        can_id,
                        data_len);
        }
    }
}
