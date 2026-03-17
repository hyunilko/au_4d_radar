#ifndef RADAR_CAN_PACKET_HANDLER_INCLUDE_H
#define RADAR_CAN_PACKET_HANDLER_INCLUDE_H

#include <condition_variable>
#include <string>
#include <atomic>
#include <cstdint>
#include <thread>
#include <queue>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <deque>
#include <memory>

#include "message_parse.hpp"   // HeaderType 가 여기 있다고 가정(기존 코드와 동일)

class PcanFdTransfer;          // ✅ 전역 forward declaration (namespace 밖)

namespace au_4d_radar
{
    class device_au_radar_node;

    class PcanLongFrameHandler
    {
    public:
        explicit PcanLongFrameHandler(device_au_radar_node* node, PcanFdTransfer& can);
        ~PcanLongFrameHandler();

        void start();
        void stop();

        int sendMessages(uint8_t device_id, uint32_t msg_id, const uint8_t* payload, int payload_len);
        std::string getRadarName(uint32_t radar_id);

    private:
        bool initialize();
        void receiveMessagesTwoQueues();
        void processMessages();
        void processClientMessages(uint32_t unique_id);
        void processPerFrameForAllSensor();

        void handleRadarScanMessage(std::vector<uint8_t>& buffer,
                                    radar_msgs::msg::RadarScan& radar_scan_msg,
                                    sensor_msgs::msg::PointCloud2& radar_cloud_msg,
                                    std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer);

        void assemblePointCloud(std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer,
                                const sensor_msgs::msg::PointCloud2& radar_cloud_msg,
                                sensor_msgs::msg::PointCloud2& multiple_cloud_messages);

        void mergePointCloud(const sensor_msgs::msg::PointCloud2& multiple_cloud_messages,
                             sensor_msgs::msg::PointCloud2& radar_cloud_msgs);

        bool isNewTimeSync(uint32_t time_sync_cloud);
        void handleRadarTrackMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarTracks& radar_tracks_msg);

    private:
        device_au_radar_node* radar_node_;
        MessageParser message_parser_;
        std::atomic<bool> receive_running;
        std::atomic<bool> process_running;
        std::atomic<bool> process_runnings;

        bool point_cloud2_setting_{false};
        uint32_t message_number_{0};

        std::thread receive_thread_;
        std::thread process_thread_;

        std::queue<std::vector<uint8_t>> message_queue_;
        std::mutex queue_mutex_;
        std::condition_variable queue_cv_;

        std::unordered_map<uint32_t, std::thread> client_threads_;
        std::mutex client_threads_mutex_;
        std::unordered_map<uint32_t, std::queue<std::vector<uint8_t>>> client_message_queues_;
        std::mutex client_queue_mutex_;
        std::unordered_map<uint32_t, std::condition_variable> client_queue_cvs_;

        sensor_msgs::msg::PointCloud2 radar_cloud_msgs;
        std::mutex publish_mutex_;
        std::mutex parse_mutex_;
        uint8_t time_sync_pre_cloud_{0};

        static constexpr size_t mTsPacketHeaderSize = 36UL;

        // ✅ CAN 전송기 — 노드 레벨에서 공유되는 PcanFdTransfer 레퍼런스
        PcanFdTransfer& can_;
    };

} // namespace au_4d_radar

#endif // RADAR_CAN_PACKET_HANDLER_INCLUDE_H