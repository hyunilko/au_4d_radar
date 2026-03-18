#include <cstring>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "util/conversion.hpp"

#include "pcan_fd_transfer.hpp"

PcanFdTransfer::PcanFdTransfer(const Config& cfg,
                               const PcanShortFrameConfig& short_cfg,
                               const PcanLongFrameConfig& long_cfg)
    : cfg_(cfg)
    , short_cfg_(short_cfg)
    , long_cfg_(long_cfg)
{
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

        print_pcan_err("CAN_WriteFD send_data", st);
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

        print_pcan_err("CAN_WriteFD send_frame64", st);
        usleep(1000);
    }

    return false;
}

bool PcanFdTransfer::send_payload(uint8_t dev_id, uint32_t msg_id, const uint8_t* payload, int payload_len)
{
    PcanLongFrame long_frame(*this, long_cfg_);
    return long_frame.send_long_payload(dev_id, msg_id, payload, payload_len);
}

bool PcanFdTransfer::send_cmd_with_data(uint8_t dev_id,
                                        ShortCanCmd cmd,
                                        uint32_t uniq_id,
                                        const uint8_t* payload,
                                        uint8_t payload_len)
{
    PcanShortFrame short_frame(*this, short_cfg_);
    return short_frame.send_short_command_with_data(dev_id, cmd, uniq_id, payload, payload_len);
}

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
