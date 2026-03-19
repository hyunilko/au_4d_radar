/**
 * @file pcan_long_frame_handler.cpp
 * @author Antonio Ko (antonioko@au-sensor.com)
 * @brief Long CAN frame receive/dispatch handler.
 * @version 1.0
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 *
 */

#include <cerrno>
#include <cstring>
#include <unistd.h>

#include "au_4d_radar.hpp"
#include "util/conversion.hpp"
#include "util/yamlParser.hpp"
#include "pcan_long_frame_handler.hpp"

namespace au_4d_radar {

/* =========================================================================
 * Construction / destruction
 * ========================================================================= */

/**
 * @brief Constructs a PcanLongFrameHandler.
 *
 * @param node Pointer to the owning radar node (for publishing and logging).
 * @param can  Reference to the PcanLongFrame layer (direct, no PcanFdTransfer indirection).
 */
PcanLongFrameHandler::PcanLongFrameHandler(device_au_radar_node* node,
                                           PcanLongFrame& can)
    : radar_node_(node)
    , can_long_(can)
    , message_parser_(node->get_logger())
{
}

/**
 * @brief Destructor. Calls stop() to cleanly join all worker threads.
 */
PcanLongFrameHandler::~PcanLongFrameHandler()
{
    stop();
}

/* =========================================================================
 * Public interface
 * ========================================================================= */

/**
 * @brief Registers the long-frame RX callback on PcanLongFrame and starts the
 *        process/client threads. Does NOT start the transport receive thread.
 *
 */
void PcanLongFrameHandler::start()
{
    if (!initialize()) {
        RCLCPP_ERROR(radar_node_->get_logger(),
                     "CAN initialization failed — start aborted.");
        return;
    }

    process_thread_running_.store(true);
    client_threads_running_.store(true);

    process_thread_ = std::thread(&PcanLongFrameHandler::processThread, this);
}

/**
 * @brief Signals all threads to exit and joins them.
 *
 * @details stop_rx() / shutdown() 은 PcanFdTransfer 소멸자가 담당한다.
 *          이 핸들러는 자신이 등록한 RX 콜백만 해제한다.
 */
void PcanLongFrameHandler::stop()
{
    process_thread_running_.store(false);
    client_threads_running_.store(false);

    queue_cv_.notify_all();

    {
        std::lock_guard<std::mutex> lk(client_queue_mutex_);
        for (auto& kv : client_queue_cvs_) {
            kv.second.notify_all();
        }
    }

    if (process_thread_.joinable()) { process_thread_.join(); }

    {
        std::lock_guard<std::mutex> lk(client_threads_mutex_);
        for (auto& kv : client_threads_) {
            if (kv.second.joinable()) { kv.second.join(); }
        }
        client_threads_.clear();
    }

    {
        std::lock_guard<std::mutex> lk(client_queue_mutex_);
        client_queue_cvs_.clear();
    }

    can_long_.set_rx_callback(PcanLongFrame::LongFrameRxCallback{});
}

/**
 * @brief Sends an application payload to a radar device via the long-frame transport.
 *
 * @param device_id   Target device index.
 * @param msg_id      Application message ID placed in the App-PDU header.
 * @param payload     Pointer to payload bytes.
 * @param payload_len Payload length in bytes.
 * @return payload_len on success, -1 on failure.
 */
int PcanLongFrameHandler::sendMessages(uint8_t device_id, uint32_t msg_id,
                                       const uint8_t* payload, int payload_len)
{
    return can_long_.send_long_payload(device_id, msg_id, payload, payload_len)
           ? payload_len
           : -1;
}

/**
 * @brief Returns the human-readable frame ID name for a given radar unique ID.
 *
 * @param radar_id 32-bit sensor unique identifier.
 * @return Frame ID string from system_info.yaml, or empty string if not found.
 */
std::string PcanLongFrameHandler::getRadarName(uint32_t radar_id)
{
    return YamlParser::getFrameIdName(radar_id);
}

/* =========================================================================
 * Private — initialization
 * ========================================================================= */

/**
 * @brief Registers the long-frame RX callback directly on PcanLongFrame.
 *
 * @details can_long_ 이 이미 PcanLongFrame& 이므로 set_rx_callback() 을 직접 호출한다.
 *
 * @return true always (PCAN init 은 PcanFdTransfer::start() 에서 담당).
 */
bool PcanLongFrameHandler::initialize()
{
    point_cloud2_enabled_ = YamlParser::getPointCloud2Setting();
    message_number_       = YamlParser::getMessageNumber();

    RCLCPP_DEBUG(radar_node_->get_logger(), "PcanLongFrameHandler::initialize()");

    /* can_long_ 이 PcanLongFrame& 이므로 직접 set_rx_callback() 호출 */
    can_long_.set_rx_callback(
        [this](uint8_t  /*dev_id*/,
               uint32_t /*frame_id*/,
               uint32_t /*frame_count*/,
               uint32_t /*msg_id*/,
               std::vector<uint8_t>&& payload)
        {
            if (payload.size() < kTsPacketHdrSize || payload.size() >= kBufferSize) {
                RCLCPP_WARN(radar_node_->get_logger(),
                            "RX callback: invalid payload size %zu", payload.size());
                return;
            }

            const uint32_t unique_id = Conversion::le_to_u32(&payload[kMsgTypeOffset]);

            if (!YamlParser::checkValidFrameId(unique_id)) {
                RCLCPP_WARN(radar_node_->get_logger(),
                            "unique_id 0x%08x not in system_info.yaml", unique_id);
                return;
            }

            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                if (message_queue_.size() >= kMaxQueueSize) {
                    message_queue_.pop();
                    RCLCPP_ERROR(radar_node_->get_logger(),
                                 "Main message queue full — discarding oldest");
                }
                message_queue_.push(std::move(payload));
            }
            queue_cv_.notify_one();
        });

    return true;
}

/* =========================================================================
 * Private — process thread (main queue → per-device dispatch)
 * ========================================================================= */

/**
 * @brief Main queue consumer — dequeues reassembled payloads and routes by unique_id.
 *
 * @details Reads the unique_id from bytes [kMsgTypeOffset … +4) (little-endian),
 *          pushes the buffer to the per-device client queue, and spawns a
 *          clientThread() for that ID on first encounter.
 */
void PcanLongFrameHandler::processThread()
{
    std::vector<uint8_t> buffer;
    buffer.reserve(kBufferSize);

    while (process_thread_running_.load()) {
        {
            std::unique_lock<std::mutex> lk(queue_mutex_);
            queue_cv_.wait(lk, [this] {
                return !message_queue_.empty() || !process_thread_running_.load();
            });

            if (!process_thread_running_.load()) {
                break;
            }

            const auto& front = message_queue_.front();
            if (front.size() < kTsPacketHdrSize || front.size() >= kBufferSize) {
                RCLCPP_WARN(radar_node_->get_logger(),
                            "processThread: invalid message size %zu — discarding",
                            front.size());
                message_queue_.pop();
                continue;
            }

            buffer = std::move(message_queue_.front());
            message_queue_.pop();
        }

        const uint32_t unique_id = Conversion::le_to_u32(&buffer[kMsgTypeOffset]);

        /* Push to per-device queue (guarded by client_queue_mutex_) */
        {
            std::lock_guard<std::mutex> lk(client_queue_mutex_);
            auto& q = client_message_queues_[unique_id];
            if (q.size() >= kMaxQueueSize) {
                q.pop();
                RCLCPP_ERROR(radar_node_->get_logger(),
                             "Client queue 0x%08x full — discarding oldest", unique_id);
            }
            q.push(buffer);
        }

        /* Spawn a client thread on first encounter */
        {
            std::lock_guard<std::mutex> lk(client_threads_mutex_);
            if (client_threads_.find(unique_id) == client_threads_.end()) {
                /* Default-construct the CV entry while we still hold
                 * client_threads_mutex_ so it exists before notify below */
                {
                    std::lock_guard<std::mutex> qlk(client_queue_mutex_);
                    (void)client_queue_cvs_[unique_id]; /* default-construct */
                }
                client_threads_.emplace(
                    unique_id,
                    std::thread(&PcanLongFrameHandler::clientThread,
                                this, unique_id));
            }
        }

        /* Notify client thread using client_queue_mutex_ (same as its wait) */
        {
            std::lock_guard<std::mutex> lk(client_queue_mutex_);
            client_queue_cvs_[unique_id].notify_one();
        }
    }
}

/* =========================================================================
 * Private — per-device client thread
 * ========================================================================= */

/**
 * @brief Per-sensor worker thread that parses and publishes messages for one unique_id.
 *
 * @details Reads the 4-byte message-type field (little-endian) and dispatches to
 *          handleScanMessage() / handlePointCloud2Message() or handleRadarTrackMessage().
 *
 * @param unique_id The sensor unique ID this thread is dedicated to.
 */
void PcanLongFrameHandler::clientThread(uint32_t unique_id)
{
    radar_msgs::msg::RadarScan    radar_scan_msg;
    radar_msgs::msg::RadarTracks  radar_tracks_msg;
    sensor_msgs::msg::PointCloud2 radar_cloud_msg;
    std::deque<sensor_msgs::msg::PointCloud2> radar_cloud_buffer;

    std::vector<uint8_t> buffer;
    buffer.reserve(kBufferSize);

    while (client_threads_running_.load()) {
        {
            std::unique_lock<std::mutex> lk(client_queue_mutex_);
            client_queue_cvs_[unique_id].wait(lk, [this, unique_id] {
                return !client_message_queues_[unique_id].empty()
                       || !client_threads_running_.load();
            });

            if (!client_threads_running_.load()) {
                break;
            }

            buffer = std::move(client_message_queues_[unique_id].front());
            client_message_queues_[unique_id].pop();
        }

        const uint32_t msg_type = Conversion::le_to_u32(buffer.data());

        switch (static_cast<HeaderType>(msg_type)) {
            case HeaderType::SCAN:
                handleScanMessage(buffer, radar_scan_msg);
                if (point_cloud2_enabled_) {
                    handlePointCloud2Message(buffer, radar_cloud_msg,
                                             radar_cloud_buffer);
                }
                break;

            case HeaderType::TRACK:
                /* handleRadarTrackMessage(buffer, radar_tracks_msg); */
                break;

            default:
                RCLCPP_WARN(radar_node_->get_logger(),
                            "clientThread: unknown msg_type 0x%08x", msg_type);
                break;
        }
    }
}

/* =========================================================================
 * Private — SCAN handler (RadarScan only)
 * ========================================================================= */

/**
 * @brief Parses a SCAN payload packet and publishes a RadarScan message when complete.
 *
 * @param buffer         Raw payload buffer.
 * @param radar_scan_msg Accumulates RadarReturn entries; cleared after each publish.
 */
void PcanLongFrameHandler::handleScanMessage(
    std::vector<uint8_t>& buffer,
    radar_msgs::msg::RadarScan& radar_scan_msg)
{
    bool complete = false;
    {
        std::lock_guard<std::mutex> lk(parse_mutex_);
        message_parser_.parseRadarScanMsg(
            &buffer[kMsgTypeOffset], radar_scan_msg, complete);
    }

    if (complete) {
        std::lock_guard<std::mutex> lk(publish_mutex_);
#ifdef DEBUG_BUILD
        const uint32_t time_ms  = radar_scan_msg.header.stamp.nanosec / 10000000u;
        const uint32_t uid      = Conversion::le_to_u32(&buffer[kMsgTypeOffset]);
        const uint32_t tpn      = Conversion::le_to_u32(&buffer[24u]);
        RCLCPP_DEBUG(radar_node_->get_logger(),
                     "UID: 0x%08x TPN: %03u  %02u ms", uid, tpn, time_ms);
#endif
        radar_node_->publishRadarScanMsg(radar_scan_msg);
        radar_scan_msg.returns.clear();
    }
}

/* =========================================================================
 * Private — PointCloud2 handler
 * ========================================================================= */

/**
 * @brief Parses a SCAN payload for PointCloud2 and publishes on a 50 ms time-sync boundary.
 *
 * @param buffer             Raw payload buffer.
 * @param radar_cloud_msg    Per-packet cloud data; cleared after assembly.
 * @param radar_cloud_buffer Rolling buffer of recent clouds for multi-frame assembly.
 */
void PcanLongFrameHandler::handlePointCloud2Message(
    std::vector<uint8_t>& buffer,
    sensor_msgs::msg::PointCloud2& radar_cloud_msg,
    std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer)
{
    bool complete = false;
    {
        std::lock_guard<std::mutex> lk(parse_mutex_);
        message_parser_.parsePointCloud2Msg(
            &buffer[kMsgTypeOffset], radar_cloud_msg, complete);
    }

    if (!complete) {
        return;
    }

    std::lock_guard<std::mutex> lk(publish_mutex_);

    sensor_msgs::msg::PointCloud2 assembled;
    assemblePointCloud(radar_cloud_buffer, radar_cloud_msg, assembled);

    const uint32_t time_sync =
        radar_cloud_msg.header.stamp.nanosec / 10000000u;

    if (isNewTimeSync(time_sync)) {
        radar_node_->publishRadarPointCloud2(radar_cloud_msgs_);
        radar_cloud_msgs_.data.clear();
        radar_cloud_msgs_              = std::move(assembled);
        radar_cloud_msgs_.header.frame_id = "RADARS";
    } else {
        mergePointCloud(assembled, radar_cloud_msgs_);
    }

    radar_cloud_msg.data.clear();
}

/* =========================================================================
 * Private — point-cloud utilities
 * ========================================================================= */

/**
 * @brief Combines a rolling window of PointCloud2 messages into one output cloud.
 *
 * @param buffer Rolling deque of recent messages (max size = message_number_).
 * @param msg    Newly completed per-packet cloud to add.
 * @param out    Output cloud combining all buffered messages.
 */
void PcanLongFrameHandler::assemblePointCloud(
    std::deque<sensor_msgs::msg::PointCloud2>& buffer,
    const sensor_msgs::msg::PointCloud2& msg,
    sensor_msgs::msg::PointCloud2& out)
{
    if (message_number_ <= 1u) {
        out = msg;
        return;
    }

    buffer.push_back(msg);
    if (buffer.size() > message_number_) {
        buffer.pop_front();
    }

    out.width       = 0u;
    out.height      = 1u;
    out.is_dense    = true;
    out.is_bigendian = false;
    out.point_step  = msg.point_step;
    out.fields      = msg.fields;
    out.header      = msg.header;
    out.data.clear();

    for (const auto& src : buffer) {
        out.width += src.width;
        out.data.insert(out.data.end(),
                        std::make_move_iterator(src.data.begin()),
                        std::make_move_iterator(src.data.end()));
    }
    out.row_step = out.point_step * out.width;
}

/**
 * @brief Appends the points from @p src into the accumulator @p dst.
 *
 * @param src Cloud to merge (width, row_step, and data are added to dst).
 * @param dst Accumulator cloud that is extended in place.
 */
void PcanLongFrameHandler::mergePointCloud(
    const sensor_msgs::msg::PointCloud2& src,
    sensor_msgs::msg::PointCloud2& dst)
{
    dst.width    += src.width;
    dst.row_step += src.row_step;
    dst.data.insert(dst.data.end(),
                    std::make_move_iterator(src.data.begin()),
                    std::make_move_iterator(src.data.end()));
}

/**
 * @brief Detects a new 50 ms time-sync window by comparing with the previous value.
 *
 * @param time_sync_cloud Current value (nanosec / 50,000,000).
 * @return true if the value changed since the last call, false if unchanged.
 */
bool PcanLongFrameHandler::isNewTimeSync(uint32_t time_sync_cloud)
{
    const bool is_new = (time_sync_pre_cloud_ != time_sync_cloud);
    time_sync_pre_cloud_ = static_cast<uint8_t>(time_sync_cloud);
    return is_new;
}

/* =========================================================================
 * Private — track handler (placeholder)
 * ========================================================================= */

/**
 * @brief Parses a TRACK payload and publishes a RadarTracks message when complete.
 *
 * @note Full track parsing is not yet implemented; parseRadarTrackMsg() is a no-op.
 *
 * @param buffer           Raw payload buffer.
 * @param radar_tracks_msg Accumulates RadarTrack entries; cleared after publish.
 */
void PcanLongFrameHandler::handleRadarTrackMessage(
    std::vector<uint8_t>& buffer,
    radar_msgs::msg::RadarTracks& radar_tracks_msg)
{
    bool complete = false;
    {
        std::lock_guard<std::mutex> lk(parse_mutex_);
        message_parser_.parseRadarTrackMsg(
            &buffer[kMsgTypeOffset], radar_tracks_msg, complete);
    }
    if (complete) {
        std::lock_guard<std::mutex> lk(publish_mutex_);
        radar_node_->publishRadarTrackMsg(radar_tracks_msg);
        radar_tracks_msg.tracks.clear();
    }
}

} // namespace au_4d_radar
