#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "message_parse.hpp"   /* MessageParser, HeaderType */
#include "pcan_long_frame.hpp"

namespace au_4d_radar
{
    class device_au_radar_node;

    class PcanLongFrameHandler
    {
    public:
        explicit PcanLongFrameHandler(device_au_radar_node* node, PcanLongFrame& can);
        ~PcanLongFrameHandler();

        void start();
        void stop();

        int sendMessages(uint8_t device_id, uint32_t msg_id, const uint8_t* payload, int payload_len);

    private:
        /* ----- lifecycle ------------------------------------------------- */
        bool initialize();

        /* ----- thread entry points --------------------------------------- */
        void processThread();
        void clientThread(uint32_t unique_id);

        /* ----- per-frame handlers (single responsibility) --------------- */
        void handleScanMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarScan& radar_scan_msg);

        void handlePointCloud2Message(
            std::vector<uint8_t>& buffer,
            sensor_msgs::msg::PointCloud2& radar_cloud_msg,
            std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer);

        /* ----- point-cloud helpers -------------------------------------- */
        void assemblePointCloud(
            std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer,
            const sensor_msgs::msg::PointCloud2& radar_cloud_msg,
            sensor_msgs::msg::PointCloud2& out);

        void mergePointCloud(const sensor_msgs::msg::PointCloud2& src, sensor_msgs::msg::PointCloud2& dst);

        bool isNewTimeSync(uint32_t time_sync_cloud);

        /* ----- track handler ------------------------------------------- */
        void handleRadarTrackMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarTracks& radar_tracks_msg);

    private:
        /* Constants */
        static constexpr size_t   kMaxQueueSize     = 1000u;
        static constexpr size_t   kTsPacketHdrSize  = 36u;
        static constexpr size_t   kBufferSize        = 5000u;
        static constexpr uint32_t kMsgTypeOffset     = 4u;

        /* Back-pointer to the owning ROS2 node */
        device_au_radar_node* radar_node_;

        /* CAN Long Frame layer */
        PcanLongFrame& can_long_;

        /* Per-instance message parser */
        MessageParser message_parser_;

        /* --- thread-lifecycle atomics ------------------------------------ */
        std::atomic<bool> process_thread_running_{false};
        std::atomic<bool> client_threads_running_{false};

        /* --- configuration cached from YamlParser ----------------------- */
        bool     point_cloud2_enabled_{false};
        uint32_t message_number_{0u};

        /* --- receive → process pipeline --------------------------------- */
        std::thread process_thread_;

        std::queue<std::vector<uint8_t>> message_queue_;
        std::mutex               queue_mutex_;
        std::condition_variable  queue_cv_;

        /* --- per-device client threads ---------------------------------- */
        /* client_queue_mutex_ guards BOTH client_message_queues_ AND
         * client_queue_cvs_, so that each CV/queue pair uses the same lock. */
        std::unordered_map<uint32_t, std::thread> client_threads_;
        std::mutex client_threads_mutex_;

        std::unordered_map<uint32_t, std::queue<std::vector<uint8_t>>> client_message_queues_;
        std::unordered_map<uint32_t, std::condition_variable>          client_queue_cvs_;
        std::mutex client_queue_mutex_;

        /* --- publish-side shared state ---------------------------------- */
        sensor_msgs::msg::PointCloud2 radar_cloud_msgs_;
        std::mutex publish_mutex_;
        std::mutex parse_mutex_;
        uint32_t    time_sync_pre_cloud_{0u};
    };

} // namespace au_4d_radar
