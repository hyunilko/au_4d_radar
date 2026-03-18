#include "pcan_long_frame.hpp"

#include <cstring>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "util/conversion.hpp"
#include "pcan_fd_transfer.hpp"

/* CustomTP wire format constants (aligned with S32 can_custom_tp.c) */
static constexpr uint8_t  HDR_PBF_MASK      = 0x80u;  /* bit7: 1=LAST, 0=MIDDLE */
static constexpr uint8_t  HDR_SEQ_HIGH_MASK = 0x3Fu;  /* bits5-0 */
static constexpr uint16_t MAX_SEQ           = 16383u; /* 14-bit */
static constexpr uint16_t CHUNK_LENGTH      = 62u;    /* 64 - 2 header */

/* App-PDU header */
static constexpr uint32_t FRAME_MAGIC_BE        = 0x12345678u;
static constexpr uint32_t APP_PDU_HEADER_LENGTH = 16u;

PcanLongFrame::PcanLongFrame(PcanFdTransfer& pcan, const Config& cfg)
    : pcan_(pcan)
    , cfg_(cfg)
{
    rx_states_.reserve(cfg_.device_count);
    for (uint8_t i = 0u; i < cfg_.device_count; ++i) {
        rx_states_.emplace_back(RxState(cfg_.rx_buf_size));
    }

    tx_frame_count_.assign(cfg_.device_count, 0u);
}

void PcanLongFrame::set_rx_callback(LongFrameRxCallback cb)
{
    rx_cb_ = std::move(cb);
}

bool PcanLongFrame::send_long_payload(uint8_t dev_id, uint32_t msg_id, const uint8_t* payload, int payload_len)
{
    if (!pcan_.initialized_ || payload == nullptr || payload_len <= 0) {
        return false;
    }
    if (dev_id >= cfg_.device_count) {
        return false;
    }

    const uint32_t frame_count = tx_frame_count_[dev_id]++;
    const uint32_t payload_size = static_cast<uint32_t>(payload_len);

    std::vector<uint8_t> app(static_cast<size_t>(payload_len) + APP_PDU_HEADER_LENGTH, 0u);
    Conversion::u32_to_be(FRAME_MAGIC_BE, &app[0]);
    Conversion::u32_to_be(frame_count, &app[4]);
    Conversion::u32_to_be(msg_id, &app[8]);
    Conversion::u32_to_be(payload_size, &app[12]);
    std::memcpy(&app[APP_PDU_HEADER_LENGTH], payload, static_cast<size_t>(payload_len));

    const uint16_t can_id = static_cast<uint16_t>(cfg_.tx_base_id + dev_id);

    uint8_t frame[64] = {0u, };
    int32_t pos = 0;
    uint16_t seq = 0u;

    while ((static_cast<int32_t>(app.size()) - pos) > static_cast<int32_t>(CHUNK_LENGTH)) {
        frame[0] = static_cast<uint8_t>((seq >> 8) & HDR_SEQ_HIGH_MASK);
        frame[1] = static_cast<uint8_t>(seq & 0xFFu);
        std::memcpy(&frame[2], &app[static_cast<size_t>(pos)], CHUNK_LENGTH);

        if (!pcan_.send_frame64(can_id, frame)) {
            return false;
        }

        pos += static_cast<int32_t>(CHUNK_LENGTH);
        seq = static_cast<uint16_t>((seq + 1u) % (MAX_SEQ + 1u));
    }

    const int remain = static_cast<int>(app.size()) - pos;
    frame[0] = static_cast<uint8_t>(HDR_PBF_MASK | ((seq >> 8) & HDR_SEQ_HIGH_MASK));
    frame[1] = static_cast<uint8_t>(seq & 0xFFu);

    if (remain <= 0) {
        frame[2] = 0x00u;
        std::memset(&frame[3], 0, CHUNK_LENGTH - 1u);
    } else {
        const int copy_n = (remain > static_cast<int>(CHUNK_LENGTH)) ? static_cast<int>(CHUNK_LENGTH) : remain;
        std::memcpy(&frame[2], &app[static_cast<size_t>(pos)], static_cast<size_t>(copy_n));
        if (copy_n < static_cast<int>(CHUNK_LENGTH)) {
            std::memset(&frame[2 + copy_n], 0, static_cast<size_t>(CHUNK_LENGTH - copy_n));
        }
    }

    return pcan_.send_frame64(can_id, frame);
}

bool PcanLongFrame::is_long_rx_can_id(uint32_t can_id, uint8_t& dev_id_out) const
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

bool PcanLongFrame::handle_long_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len)
{
    uint8_t dev_id = 0u;
    if (!is_long_rx_can_id(can_id, dev_id)) {
        return false;
    }

    process_long_tp_frame(dev_id, data, data_len);
    return true;
}

void PcanLongFrame::process_long_tp_frame(uint8_t dev_id, const uint8_t* data, uint8_t data_len)
{
    if (data == nullptr || dev_id >= rx_states_.size() || data_len < 2u) {
        return;
    }

    RxState& st = rx_states_[dev_id];
    const uint8_t hdr1 = data[0];
    const uint8_t hdr2 = data[1];
    const bool is_last = ((hdr1 & HDR_PBF_MASK) != 0u);
    const uint16_t seq = static_cast<uint16_t>(((hdr1 & HDR_SEQ_HIGH_MASK) << 8) | hdr2);
    const uint16_t expect = st.seq_expect;

    if (seq != expect) {
        const uint16_t prev = (expect == 0u) ? MAX_SEQ : static_cast<uint16_t>(expect - 1u);

        if (!is_last && (seq == prev)) {
            return;
        }

        if (seq == 0u) {
            st.reset();
        } else {
            st.reset();
            RCLCPP_ERROR(rclcpp::get_logger("PcanLongFrame"),
                         "Long TP seq mismatch dev=%u seq=%u expect=%u",
                         dev_id, seq, expect);
            return;
        }
    }

    if ((st.len + CHUNK_LENGTH) > st.buf.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("PcanLongFrame"),
                     "Long TP buffer overflow dev=%u len=%u buf_size=%zu",
                     dev_id, st.len, st.buf.size());
        st.reset();
        return;
    }

    const uint8_t* payload = &data[2];
    const int avail = static_cast<int>(data_len) - 2;
    if (avail >= static_cast<int>(CHUNK_LENGTH)) {
        std::memcpy(st.buf.data() + st.len, payload, CHUNK_LENGTH);
    } else if (avail > 0) {
        std::memcpy(st.buf.data() + st.len, payload, static_cast<size_t>(avail));
        std::memset(st.buf.data() + st.len + avail, 0, static_cast<size_t>(CHUNK_LENGTH - avail));
    } else {
        std::memset(st.buf.data() + st.len, 0, CHUNK_LENGTH);
    }
    st.len += CHUNK_LENGTH;

    if (!is_last) {
        st.seq_expect = static_cast<uint16_t>((st.seq_expect + 1u) % (MAX_SEQ + 1u));
        return;
    }

    if (st.len < APP_PDU_HEADER_LENGTH) {
        st.reset();
        RCLCPP_ERROR(rclcpp::get_logger("PcanLongFrame"), "Long TP short AppPDU dev=%u", dev_id);
        return;
    }

    const uint32_t frame_id = Conversion::be_to_u32(&st.buf[0]);
    const uint32_t frame_count = Conversion::be_to_u32(&st.buf[4]);
    const uint32_t msg_id = Conversion::be_to_u32(&st.buf[8]);
    const uint32_t payload_len = Conversion::be_to_u32(&st.buf[12]);
    const uint64_t needed = APP_PDU_HEADER_LENGTH + static_cast<uint64_t>(payload_len);

    if (frame_id != FRAME_MAGIC_BE) {
        if (!cfg_.quiet) {
            RCLCPP_ERROR(rclcpp::get_logger("PcanLongFrame"),
                         "[Long TP] bad frame magic dev=%u frame_id=0x%08X frame_count=%u",
                         dev_id, frame_id, frame_count);
        }
        st.reset();
        return;
    }

    if ((needed > st.buf.size()) || (needed > st.len)) {
        if (!cfg_.quiet) {
            RCLCPP_ERROR(rclcpp::get_logger("PcanLongFrame"),
                         "[Long TP] length mismatch dev=%u need=%llu have=%u",
                         dev_id, static_cast<unsigned long long>(needed), st.len);
        }
        st.reset();
        return;
    }

    std::vector<uint8_t> payload_out(payload_len, 0u);
    if (payload_len > 0u) {
        std::memcpy(payload_out.data(), &st.buf[APP_PDU_HEADER_LENGTH], payload_len);
    }

    if (rx_cb_) {
        rx_cb_(dev_id, frame_id, frame_count, msg_id, std::move(payload_out));
    }

    st.reset();
}

/* PcanFdTransfer long wrapper implementations */
void PcanFdTransfer::set_long_rx_callback(LongRxCallback cb)
{
    if (long_frame_) {
        long_frame_->set_rx_callback(std::move(cb));
    }
}

bool PcanFdTransfer::send_payload(uint8_t dev_id, uint32_t msg_id, const uint8_t* payload, int payload_len)
{
    return (long_frame_ != nullptr)
        ? long_frame_->send_long_payload(dev_id, msg_id, payload, payload_len)
        : false;
}
