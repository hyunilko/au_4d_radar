/**
 * @file radar_packet_handler.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief Implementation of the radar_can_packet_handler class for processing incoming radar data.
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
#include "radar_can_packet_handler.hpp"

#include "pcan_fd_transfer.h"

#define BUFFER_SIZE     2048
#define MSG_TYPE_OFFSET 4
const size_t MAX_QUEUE_SIZE = 1000;

enum HeaderType {
    HEADER_SCAN = 0x5343414e,
    HEADER_TRACK = 0x54524143,
    HEADER_MON = 0x4d4f4e49
};

namespace au_4d_radar {

RadarCanPacketHandler::RadarCanPacketHandler(device_au_radar_node* node)
    : radar_node_(node),
      message_parser_(node->get_logger()),
      receive_running(true),
      process_running(true),
      process_runnings(true)
{
}

RadarCanPacketHandler::~RadarCanPacketHandler()
{
    stop();
}

void RadarCanPacketHandler::start()
{
    if (initialize())
    {
        receive_thread_ = std::thread(&RadarCanPacketHandler::receiveMessagesTwoQueues, this);
        process_thread_ = std::thread(&RadarCanPacketHandler::processMessages, this);
    }
    else
    {
        RCLCPP_ERROR(radar_node_->get_logger(), "CAN initialization failed, start aborted.");
    }
}

void RadarCanPacketHandler::stop()
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

    if (can_)
    {
        can_->set_rx_callback(PcanFdTransfer::RxCallback{}); // nullptr 대신 빈 콜백
        can_->shutdown();
        can_.reset();
    }
}

bool RadarCanPacketHandler::initialize()
{
    point_cloud2_setting_ = YamlParser::readPointCloud2Setting("POINT_CLOUD2");
    message_number_       = YamlParser::readMessageNumber("MESSAGE_NUMBER");

    // ✅ CAN init
    PcanFdTransfer::Config cfg;
    cfg.handle       = PCAN_USBBUS1;
    cfg.device_count = 4;
    cfg.tx_base_id   = 0x600;   // PC -> S32
    cfg.rx_base_id   = 0x650;   // S32 -> PC (현재 S32 app_can_send 구현 기준)
    cfg.brs_on       = true;
    cfg.quiet        = true;

    RCLCPP_DEBUG(radar_node_->get_logger(), "RadarCanPacketHandler::initialize()");

    can_ = std::make_unique<PcanFdTransfer>(cfg);
    if (!can_->init())
    {
        RCLCPP_ERROR(radar_node_->get_logger(), "PCAN init failed");
        return false;
    }

    // ✅ 수신된 TP 메시지 payload를 message_queue_에 push
    // (새 pcan_fd_transfer: dev_id, frame_id, frame_count, payload)
    can_->set_rx_callback([this](uint8_t dev_id,
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
        //RCLCPP_INFO(radar_node_->get_logger(), "unique_id %08x", unique_id);
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

void RadarCanPacketHandler::receiveMessagesTwoQueues()
{
    while (receive_running.load())
    {
        if (can_)
        {
            can_->poll_rx();
        }
        usleep(1000);
    }
}

void RadarCanPacketHandler::processMessages() {
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
                client_threads_[unique_id] = std::thread(&RadarCanPacketHandler::processClientMessages, this, unique_id);
            }
        }
        buffer.clear();
        client_queue_cvs_[unique_id].notify_one();
    }
}

void RadarCanPacketHandler::processClientMessages(uint32_t unique_id) {
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

void RadarCanPacketHandler::handleRadarScanMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarScan& radar_scan_msg,
        sensor_msgs::msg::PointCloud2& radar_cloud_msg, std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer) {

    {
        bool completeRadarScanMsg = false;
        {
            std::lock_guard<std::mutex> lock(parse_mutex_);
            message_parser_.parseRadarScanMsg(&buffer[MSG_TYPE_OFFSET], radar_scan_msg, completeRadarScanMsg);
        }

        if (completeRadarScanMsg) {
            std::lock_guard<std::mutex> lock(publish_mutex_);
            //uint32_t time_sync_scan = radar_scan_msg.header.stamp.nanosec / 10000000;
            //RCLCPP_DEBUG(radar_node_->get_logger(), "id: %s 10ms %02u", radar_scan_msg.header.frame_id.c_str(), time_sync_scan);
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
void RadarCanPacketHandler::assemblePointCloud(std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer,
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
void RadarCanPacketHandler::mergePointCloud(const sensor_msgs::msg::PointCloud2& multiple_cloud_messages,
        sensor_msgs::msg::PointCloud2& radar_cloud_msgs) {

    radar_cloud_msgs.width += multiple_cloud_messages.width;
    radar_cloud_msgs.row_step += multiple_cloud_messages.row_step;
    radar_cloud_msgs.data.insert(radar_cloud_msgs.data.end(),
                                 std::make_move_iterator(multiple_cloud_messages.data.begin()),
                                 std::make_move_iterator(multiple_cloud_messages.data.end()));
}

bool RadarCanPacketHandler::isNewTimeSync(uint32_t time_sync_cloud) {
    bool isNew = (time_sync_pre_cloud_ != time_sync_cloud);
    time_sync_pre_cloud_ = time_sync_cloud;
    return isNew;
}

void RadarCanPacketHandler::handleRadarTrackMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarTracks& radar_tracks_msg) {
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

int RadarCanPacketHandler::sendMessages(uint8_t device_id, uint32_t msg_id,
                                        const uint8_t* payload, int payload_len)
{
    (void)msg_id;
    if (!can_) return -1;

    return can_->send_payload(device_id, msg_id, payload, payload_len) ? payload_len : -1;
}

}  // namespace au_4d_radar