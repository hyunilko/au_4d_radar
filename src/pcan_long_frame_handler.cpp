/**
 * @file pcan_long_frame_handler.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief Implementation of the pcan_long_frame_handler class for processing incoming radar data.
 * @version 1.0
 * @date 2026-03-09
 *
 * @copyright Copyright AU (c) 2024
 *
 */

#include <cerrno>
#include <cstring>
#include <unistd.h>

#include "au_4d_radar.hpp"
#include "util/conversion.hpp"
#include "util/yamlParser.hpp"
#include "pcan_long_frame_handler.hpp"

#include "pcan_fd_transfer.hpp"


#define BUFFER_SIZE     2048
#define MSG_TYPE_OFFSET 4
const size_t MAX_QUEUE_SIZE = 1000;

enum HeaderType {
    HEADER_SCAN = 0x5343414e,
    HEADER_TRACK = 0x54524143,
    HEADER_MON = 0x4d4f4e49
};

namespace au_4d_radar {

/**
 * @brief Constructs a PcanLongFrameHandler.
 *
 * @param node Pointer to the owning ROS2 radar node (used for publishing and logging).
 * @param can  Reference to the transport layer for sending and receiving CAN frames.
 */
PcanLongFrameHandler::PcanLongFrameHandler(device_au_radar_node* node, PcanFdTransfer& can)
    : radar_node_(node),
      message_parser_(node->get_logger()),
      receive_running(true),
      process_running(true),
      process_runnings(true),
      can_(can),
      long_frame_(can, can.long_frame_config())
{
}

/**
 * @brief Destructor. Calls stop() to cleanly terminate all worker threads.
 */
PcanLongFrameHandler::~PcanLongFrameHandler()
{
    stop();
}

/**
 * @brief Initialises the PCAN interface and starts the receive and process threads.
 *
 * @details Calls initialize() to open the CAN channel and install the RX callback,
 *          then spawns the receiveMessagesTwoQueues() and processMessages() threads.
 *          Logs an error and returns without starting threads if initialisation fails.
 */
void PcanLongFrameHandler::start()
{
    if (initialize())
    {
        receive_thread_ = std::thread(&PcanLongFrameHandler::receiveMessagesTwoQueues, this);
        process_thread_ = std::thread(&PcanLongFrameHandler::processMessages, this);
    }
    else
    {
        RCLCPP_ERROR(radar_node_->get_logger(), "CAN initialization failed, start aborted.");
    }
}

/**
 * @brief Signals all worker threads to stop, joins them, and shuts down the CAN interface.
 */
void PcanLongFrameHandler::stop()
{
    receive_running.store(false);
    process_running.store(false);
    process_runnings.store(false);

    queue_cv_.notify_all();
    {
        std::lock_guard<std::mutex> lock(client_threads_mutex_);
        for (auto& pair : client_queue_cvs_)
            pair.second.notify_all();
    }

    if (receive_thread_.joinable()) receive_thread_.join();
    if (process_thread_.joinable()) process_thread_.join();

    {
        std::lock_guard<std::mutex> lock(client_threads_mutex_);
        for (auto& pair : client_threads_)
            if (pair.second.joinable()) pair.second.join();

        client_threads_.clear();
        client_queue_cvs_.clear();
    }

    long_frame_.set_rx_callback(PcanLongFrame::LongFrameRxCallback{});
    can_.shutdown();
}

/**
 * @brief Opens the PCAN channel and installs the long-frame RX callback.
 *
 * @details Reads YAML settings (point-cloud mode, message number), calls
 *          PcanFdTransfer::init(), and registers the lambda that validates
 *          incoming payloads and pushes them onto the shared message queue.
 *
 * @return true  on successful initialisation.
 * @return false if PCAN init fails.
 */
bool PcanLongFrameHandler::initialize()
{
    point_cloud2_setting_ = YamlParser::readPointCloud2Setting("POINT_CLOUD2");
    message_number_       = YamlParser::readMessageNumber("MESSAGE_NUMBER");

    RCLCPP_DEBUG(radar_node_->get_logger(), "PcanLongFrameHandler::initialize()");

    if (!can_.init())
    {
        RCLCPP_ERROR(radar_node_->get_logger(), "PCAN init failed");
        return false;
    }

    long_frame_.set_rx_callback([this](uint8_t dev_id,
                                 uint32_t frame_id,
                                 uint32_t frame_count,
                                 uint32_t msg_id,
                                 std::vector<uint8_t>&& payload)
    {
       (void)dev_id;
       (void)frame_id;
       (void)frame_count;
       (void)msg_id;

        if (payload.size() < mTsPacketHeaderSize || payload.size() >= BUFFER_SIZE)
        {
            RCLCPP_WARN(radar_node_->get_logger(), "Invalid payload size: %zu", payload.size());
            return;
        }

        const uint32_t unique_id = Conversion::le_to_u32(&payload[MSG_TYPE_OFFSET]);
        //RCLCPP_INFO(radar_node_->get_logger(), "dev_id: %d frame_id %08x frame_count %u msg_id %08x unique_id %08x", dev_id, frame_id, frame_count, msg_id, unique_id);
        if (!YamlParser::checkValidFrameId(unique_id))
        {
            RCLCPP_WARN(radar_node_->get_logger(), "unique_id %08x not in system_info.yaml", unique_id);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (message_queue_.size() >= MAX_QUEUE_SIZE)
            {
                message_queue_.pop();
                RCLCPP_ERROR(radar_node_->get_logger(), "Message queue full, discard oldest");
            }
            message_queue_.push(std::move(payload));
        }
        queue_cv_.notify_one();
    });

    return true;
}

/**
 * @brief Lifecycle thread kept for compatibility; RX is handled by pcanRxDispatchLoop.
 *
 * @details The actual CAN receive pump runs in device_au_radar_node::pcanRxDispatchLoop
 *          which calls handle_can_frame() directly. This thread simply sleeps until
 *          receive_running is cleared.
 */
void PcanLongFrameHandler::receiveMessagesTwoQueues()
{
    /* RX polling is handled by pcanRxDispatchLoop in device_au_radar_node.
     * This thread is kept for lifecycle compatibility only. */
    while (receive_running.load()) {
        usleep(100000);
    }
}

/**
 * @brief Dispatches an incoming CAN frame to the owned PcanLongFrame assembler.
 *
 * @param can_id   CAN ID of the received frame.
 * @param data     Pointer to the frame payload bytes.
 * @param data_len Length of the frame payload.
 * @return true  if the frame was consumed by the long-frame handler.
 * @return false if the CAN ID does not belong to the long-frame RX range.
 */
bool PcanLongFrameHandler::handle_can_frame(uint32_t can_id, const uint8_t* data, uint8_t data_len)
{
    return long_frame_.handle_long_can_frame(can_id, data, data_len);
}

/**
 * @brief Consumer thread that dequeues reassembled payloads and routes them by unique_id.
 *
 * @details For each payload, reads the unique_id from bytes 4–7 (little-endian), pushes
 *          the buffer into the per-client queue, and spawns a dedicated processClientMessages
 *          thread for that unique_id if one is not already running.
 */
void PcanLongFrameHandler::processMessages() {
    std::vector<uint8_t> buffer(BUFFER_SIZE);
    while (process_running.load()) {
        // std::vector<uint8_t> buffer(BUFFER_SIZE);
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !message_queue_.empty() || !process_running.load(); });

            if (!process_running.load()) {
                break;
            }

            if (message_queue_.front().size() < mTsPacketHeaderSize || message_queue_.front().size() >= BUFFER_SIZE) {
                RCLCPP_WARN(rclcpp::get_logger("processMessages"), "Invalid message size detected and discarded.");
                message_queue_.pop();
                continue;
            }

            buffer = std::move(message_queue_.front());
            message_queue_.pop();
        }

        uint32_t unique_id = Conversion::le_to_u32(&buffer[MSG_TYPE_OFFSET]);

        {
            std::lock_guard<std::mutex> lock(client_queue_mutex_);
            if (client_message_queues_[unique_id].size() >= MAX_QUEUE_SIZE) {
                client_message_queues_[unique_id].pop();
                RCLCPP_ERROR(rclcpp::get_logger("processMessages"), "Client message queue is full, discarding oldest message");
            }
            client_message_queues_[unique_id].push(buffer);
        }

        {
            std::lock_guard<std::mutex> lock(client_threads_mutex_);
            if (client_threads_.find(unique_id) == client_threads_.end()) {
                client_queue_cvs_[unique_id];
                client_threads_[unique_id] = std::thread(&PcanLongFrameHandler::processClientMessages, this, unique_id);
            }
        }
        buffer.clear();
        client_queue_cvs_[unique_id].notify_one();
    }
}

/**
 * @brief Per-client worker thread that dequeues and parses messages for one unique_id.
 *
 * @details Reads the 4-byte message-type field (bytes 0–3, little-endian) and dispatches
 *          to handleRadarScanMessage() or handleRadarTrackMessage() based on the HeaderType.
 *
 * @param unique_id The sensor unique ID this thread is dedicated to processing.
 */
void PcanLongFrameHandler::processClientMessages(uint32_t unique_id) {
    radar_msgs::msg::RadarScan radar_scan_msg;
    sensor_msgs::msg::PointCloud2 radar_cloud_msg;
    radar_msgs::msg::RadarTracks radar_tracks_msg;
    std::deque<sensor_msgs::msg::PointCloud2> radar_cloud_buffer;

    std::vector<uint8_t> buffer(BUFFER_SIZE);

    while (process_runnings.load()) {
        // std::vector<uint8_t> buffer(BUFFER_SIZE);
        {
            std::unique_lock<std::mutex> lock(client_queue_mutex_);
            client_queue_cvs_[unique_id].wait(lock, [this, &unique_id] {
                return !client_message_queues_[unique_id].empty() || !process_runnings.load();
            });

            if (!process_runnings.load()) {
                break;
            }

            buffer = std::move(client_message_queues_[unique_id].front());
            client_message_queues_[unique_id].pop();
        }

        uint32_t msg_type = Conversion::le_to_u32(buffer.data());

        switch (msg_type) {
            case HeaderType::HEADER_SCAN:
                handleRadarScanMessage(buffer, radar_scan_msg, radar_cloud_msg, radar_cloud_buffer);
                break;
            case HeaderType::HEADER_TRACK:
                //handleRadarTrackMessage(buffer, radar_tracks_msg);
                break;
            default:
                RCLCPP_WARN(rclcpp::get_logger("processClientMessages"), "Unknown message type: %08x", msg_type);
                break;
        }
        buffer.clear();
    }
}

/**
 * @brief Parses a HEADER_SCAN payload and publishes RadarScan and (optionally) PointCloud2.
 *
 * @param buffer             Raw payload buffer (modified via move on queue pop).
 * @param radar_scan_msg     Accumulates RadarReturn entries across packets; cleared after publish.
 * @param radar_cloud_msg    Accumulates point-cloud data; cleared after assembly.
 * @param radar_cloud_buffer Rolling buffer of recent PointCloud2 messages for multi-frame assembly.
 */
void PcanLongFrameHandler::handleRadarScanMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarScan& radar_scan_msg,
        sensor_msgs::msg::PointCloud2& radar_cloud_msg, std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer) {

    {
        bool completeRadarScanMsg = false;
        {
            std::lock_guard<std::mutex> lock(parse_mutex_);
            message_parser_.parseRadarScanMsg(&buffer[MSG_TYPE_OFFSET], radar_scan_msg, completeRadarScanMsg);
        }

        if (completeRadarScanMsg) {
            std::lock_guard<std::mutex> lock(publish_mutex_);
#ifdef DEBUG_BUILD
            uint32_t time_sync_scan = radar_scan_msg.header.stamp.nanosec / 10000000;
            uint32_t UID = Conversion::le_to_u32(&buffer[MSG_TYPE_OFFSET]);
            uint32_t TPN = Conversion::le_to_u32(&buffer[24]);
            RCLCPP_DEBUG(radar_node_->get_logger(), "UID: %08x TPN: %03u %02u ms", UID, TPN, time_sync_scan);
#endif
            radar_node_->publishRadarScanMsg(radar_scan_msg);
            radar_scan_msg.returns.clear();
        }
    }

    if (point_cloud2_setting_) {
        bool completePointCloud2Msg = false;
        {
            std::lock_guard<std::mutex> lock(parse_mutex_);
            message_parser_.parsePointCloud2Msg(&buffer[MSG_TYPE_OFFSET], radar_cloud_msg, completePointCloud2Msg);
        }

        if (completePointCloud2Msg) {
            std::lock_guard<std::mutex> lock(publish_mutex_);
            sensor_msgs::msg::PointCloud2 multiple_cloud_messages;
            assemblePointCloud(radar_cloud_buffer, radar_cloud_msg, multiple_cloud_messages);

            uint32_t time_sync_cloud = radar_cloud_msg.header.stamp.nanosec / 10000000;
            if (isNewTimeSync(time_sync_cloud)) {
                //RCLCPP_DEBUG(radar_node_->get_logger(), "id: RADARS 10ms %02u", time_sync_cloud);
                radar_node_->publishRadarPointCloud2(radar_cloud_msgs);
                radar_cloud_msgs.data.clear();
                radar_cloud_msgs = std::move(multiple_cloud_messages);
                radar_cloud_msgs.header.frame_id = "RADARS";
            } else {
                mergePointCloud(multiple_cloud_messages, radar_cloud_msgs);
            }
            radar_cloud_msg.data.clear();
        }
    }
}

/**
 * @brief function to assemble point cloud messages into a single message
 *
 * @param radar_cloud_buffer
 * @param radar_cloud_msg
 * @param multiple_cloud_messages
 */
void PcanLongFrameHandler::assemblePointCloud(std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer,
        const sensor_msgs::msg::PointCloud2& radar_cloud_msg, sensor_msgs::msg::PointCloud2& multiple_cloud_messages) {

    if (message_number_ > 1) {
        radar_cloud_buffer.push_back(radar_cloud_msg);
        if (radar_cloud_buffer.size() > message_number_) {
            radar_cloud_buffer.pop_front();
        }

        multiple_cloud_messages.width = 0;
        multiple_cloud_messages.height = 1;
        multiple_cloud_messages.is_dense = true;
        multiple_cloud_messages.is_bigendian = false;
        multiple_cloud_messages.point_step = radar_cloud_msg.point_step;
        multiple_cloud_messages.fields = radar_cloud_msg.fields;
        multiple_cloud_messages.header = radar_cloud_msg.header;

        for (const auto& msg : radar_cloud_buffer) {
            multiple_cloud_messages.width += msg.width;
            multiple_cloud_messages.data.insert(multiple_cloud_messages.data.end(),
                                                std::make_move_iterator(msg.data.begin()),
                                                std::make_move_iterator(msg.data.end()));
        }
        multiple_cloud_messages.row_step = multiple_cloud_messages.point_step * multiple_cloud_messages.width;
    } else {
        multiple_cloud_messages = radar_cloud_msg;
    }
}

/**
 * @brief function to merge multiple point cloud messages
 *
 * @param multiple_cloud_messages
 * @param radar_cloud_msgs
 */
void PcanLongFrameHandler::mergePointCloud(const sensor_msgs::msg::PointCloud2& multiple_cloud_messages,
        sensor_msgs::msg::PointCloud2& radar_cloud_msgs) {

    radar_cloud_msgs.width += multiple_cloud_messages.width;
    radar_cloud_msgs.row_step += multiple_cloud_messages.row_step;
    radar_cloud_msgs.data.insert(radar_cloud_msgs.data.end(),
                                 std::make_move_iterator(multiple_cloud_messages.data.begin()),
                                 std::make_move_iterator(multiple_cloud_messages.data.end()));
}

/**
 * @brief Detects whether the 10 ms time-sync bucket has changed since the last call.
 *
 * @param time_sync_cloud Current time-sync value (nanosec / 10,000,000).
 * @return true  if the value differs from the previous call (new sync window).
 * @return false if the value is unchanged.
 */
bool PcanLongFrameHandler::isNewTimeSync(uint32_t time_sync_cloud) {
    bool isNew = (time_sync_pre_cloud_ != time_sync_cloud);
    time_sync_pre_cloud_ = time_sync_cloud;
    return isNew;
}

/**
 * @brief Parses a HEADER_TRACK payload and publishes a RadarTracks message.
 *
 * @param buffer           Raw payload buffer.
 * @param radar_tracks_msg Accumulates RadarTrack entries; cleared after publish.
 */
void PcanLongFrameHandler::handleRadarTrackMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarTracks& radar_tracks_msg) {
    bool completeRadarTrackMsg = false;
    {
        std::lock_guard<std::mutex> lock(parse_mutex_);
        message_parser_.parseRadarTrackMsg(&buffer[MSG_TYPE_OFFSET], radar_tracks_msg, completeRadarTrackMsg);
    }
    if (completeRadarTrackMsg) {
        std::lock_guard<std::mutex> lock(publish_mutex_);
        radar_node_->publishRadarTrackMsg(radar_tracks_msg);
        radar_tracks_msg.tracks.clear();
    }
}

/**
 * @brief Sends an application payload to a radar device via the long-frame transport.
 *
 * @param device_id   Target device index.
 * @param msg_id      Application message ID placed in the App-PDU header.
 * @param payload     Pointer to the payload bytes.
 * @param payload_len Length of the payload in bytes.
 * @return payload_len on success, -1 on failure.
 */
int PcanLongFrameHandler::sendMessages(uint8_t device_id, uint32_t msg_id,
                                        const uint8_t* payload, int payload_len)
{
    (void)msg_id;
    return can_.send_payload(device_id, msg_id, payload, payload_len) ? payload_len : -1;
}

}  // namespace au_4d_radar