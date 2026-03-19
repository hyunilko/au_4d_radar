/**
 * @file pcan_long_frame.cpp
 * @author antonioko@au-sensor.com
 * @brief CustomTP reassembler/transmitter for large CAN-FD payloads.
 * @version 1.0
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 *
 * @details Splits application payloads into 64-byte CAN-FD chunks. Each chunk
 *          carries a 2-byte header: PBF flag (bit 7) + 14-bit sequence number,
 *          followed by 62 bytes of data. The final chunk has PBF = 1.
 *          Reassembled payloads are delivered via a LongFrameRxCallback.
 */

#include "pcan_long_frame.hpp"

#include <cstring>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "util/conversion.hpp"
#include "pcan_fd_transfer.hpp"

/* CustomTP wire format constants */
static constexpr uint8_t  HDR_PBF_MASK      = 0x80u;  /* bit7: 1=LAST, 0=MIDDLE */
static constexpr uint8_t  HDR_SEQ_HIGH_MASK = 0x3Fu;  /* bits5-0 */
static constexpr uint16_t MAX_SEQ           = 16383u; /* 14-bit */
static constexpr uint16_t CHUNK_LENGTH      = 62u;    /* 64 - 2 header */

/* App-PDU header */
static constexpr uint32_t FRAME_MAGIC_BE        = 0x12345678u;
static constexpr uint32_t APP_PDU_HEADER_LENGTH = 16u;

/**
 * @brief Constructs a PcanLongFrame and allocates per-device RX reassembly buffers.
 *
 * @param pcan Reference to the transport layer used for TX.
 * @param cfg  Long-frame configuration (CAN IDs, device count, RX buffer size).
 */
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

/**
 * @brief Registers the callback invoked when a complete App-PDU is reassembled.
 *
 * @param cb Callback: void(dev_id, frame_id, frame_count, msg_id, payload&&).
 *           Pass an empty std::function to deregister.
 */
void PcanLongFrame::set_rx_callback(LongFrameRxCallback cb)
{
    rx_cb_ = std::move(cb);
}

/**
 * @brief Transmits a large payload using the CustomTP chunking protocol.
 *
 * @details Prepends a 16-byte App-PDU header (magic, frame_count, msg_id, payload_len)
 *          and splits the result into 64-byte CAN-FD frames. Intermediate frames have
 *          PBF = 0; the final frame sets PBF = 1 (bit 7 of byte 0).
 *
 * @param dev_id      Target device index (0 … device_count-1).
 * @param msg_id      Application message ID placed in the App-PDU header.
 * @param payload     Pointer to application data.
 * @param payload_len Payload length in bytes (must be > 0).
 * @return true if all frames were sent, false on invalid args or send error.
 */
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

    std::vector<uint8_t> buff(static_cast<size_t>(payload_len) + APP_PDU_HEADER_LENGTH, 0u);
    Conversion::u32_to_be(FRAME_MAGIC_BE, &buff[0]);
    Conversion::u32_to_be(frame_count, &buff[4]);
    Conversion::u32_to_be(msg_id, &buff[8]);
    Conversion::u32_to_be(payload_size, &buff[12]);
    std::memcpy(&buff[APP_PDU_HEADER_LENGTH], payload, static_cast<size_t>(payload_len));

    const uint16_t can_id = static_cast<uint16_t>(cfg_.tx_base_id + dev_id);

    uint8_t frame[64] = {0u, };
    int32_t pos = 0;
    uint16_t seq = 0u;

    while ((static_cast<int32_t>(buff.size()) - pos) > static_cast<int32_t>(CHUNK_LENGTH)) {
        frame[0] = static_cast<uint8_t>((seq >> 8) & HDR_SEQ_HIGH_MASK);
        frame[1] = static_cast<uint8_t>(seq & 0xFFu);
        std::memcpy(&frame[2], &buff[static_cast<size_t>(pos)], CHUNK_LENGTH);

        if (!pcan_.send_frame64(can_id, frame)) {
            return false;
        }

        pos += static_cast<int32_t>(CHUNK_LENGTH);
        seq = static_cast<uint16_t>((seq + 1u) % (MAX_SEQ + 1u));
    }

    const int remain = static_cast<int>(buff.size()) - pos;
    frame[0] = static_cast<uint8_t>(HDR_PBF_MASK | ((seq >> 8) & HDR_SEQ_HIGH_MASK));
    frame[1] = static_cast<uint8_t>(seq & 0xFFu);

    if (remain <= 0) {
        frame[2] = 0x00u;
        std::memset(&frame[3], 0, CHUNK_LENGTH - 1u);
    } else {
        const int copy_n = (remain > static_cast<int>(CHUNK_LENGTH)) ? static_cast<int>(CHUNK_LENGTH) : remain;
        std::memcpy(&frame[2], &buff[static_cast<size_t>(pos)], static_cast<size_t>(copy_n));
        if (copy_n < static_cast<int>(CHUNK_LENGTH)) {
            std::memset(&frame[2 + copy_n], 0, static_cast<size_t>(CHUNK_LENGTH - copy_n));
        }
    }

    return pcan_.send_frame64(can_id, frame);
}

/**
 * @brief Checks whether a CAN ID belongs to the long-frame RX range.
 *
 * @param can_id      Received CAN ID.
 * @param dev_id_out  Set to the device index when the function returns true.
 * @return true  if can_id maps to a valid device, false otherwise.
 */
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

/**
 * @brief Dispatches a CAN frame to the reassembler if its ID is in the long-frame RX range.
 *
 * @param can_id   CAN ID of the received frame.
 * @param data     Frame payload bytes.
 * @param data_len Payload length.
 * @return true if handled, false if the CAN ID is out of range.
 */
bool PcanLongFrame::handle_long_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len)
{
    uint8_t dev_id = 0u;
    if (!is_long_rx_can_id(can_id, dev_id)) {
        return false;
    }

    process_long_tp_frame(dev_id, data, data_len);
    return true;
}

/**
 * @brief Reassembles one CustomTP chunk into the per-device RX buffer.
 *
 * @details Validates the sequence number, appends the 62-byte chunk, and — on
 *          receipt of the last chunk (PBF = 1) — validates the App-PDU magic and
 *          payload length, then fires the LongFrameRxCallback.
 *
 * @param dev_id   Device index derived from the received CAN ID.
 * @param data     Raw 64-byte CAN-FD frame payload.
 * @param data_len Actual payload length (must be ≥ 2).
 */
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
    const int avail = static_cast<int>(data_len) - 2; // data_len - header_len
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

