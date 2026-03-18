#ifndef MESSAGE_PARSE_HPP
#define MESSAGE_PARSE_HPP

#include <cstdint>
#include <string>
#include <unordered_map>

#include <boost/uuid/uuid.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <radar_msgs/msg/radar_return.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_track.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "util/yamlParser.hpp"

namespace au_4d_radar
{

/* -----------------------------------------------------------------------
 * Wire message types (first 4 bytes of every long-frame payload, LE).
 * Defined here so handlers can switch on them without including
 * a separate header.
 * ----------------------------------------------------------------------- */
enum class HeaderType : uint32_t
{
    SCAN  = 0x5343414Eu,  /* 'SCAN' */
    TRACK = 0x54524143u,  /* 'TRAC' */
    MON   = 0x4D4F4E49u,  /* 'MONI' */
};

class MessageParser
{
public:
    explicit MessageParser(rclcpp::Logger logger =
                               rclcpp::get_logger("MessageParser"))
        : logger_(logger) {}
    ~MessageParser() = default;

    /* Each public method is NOT internally locked — callers are expected
     * to hold their own serialisation mutex (parse_mutex_ in the handler).
     * This removes the redundant double-lock that existed previously. */
    void parsePointCloud2Msg(uint8_t* p_buff,
                             sensor_msgs::msg::PointCloud2& cloud_msg,
                             bool& complete);

    void parseRadarScanMsg(uint8_t* p_buff,
                           radar_msgs::msg::RadarScan& radar_scan_msg,
                           bool& complete);

    void parseRadarTrackMsg(uint8_t* p_buff,
                            radar_msgs::msg::RadarTracks& radar_tracks_msg,
                            bool& complete);

private:
    void makeRadarPointCloud2Msg(uint8_t* p_buff,
                                 sensor_msgs::msg::PointCloud2& cloud_msg,
                                 bool& complete);

    void makeRadarScanMsg(uint8_t* p_buff,
                          radar_msgs::msg::RadarScan& radar_scan_msg,
                          bool& complete);

    void makeRadarTracksMsg(uint8_t* p_buff,
                            radar_msgs::msg::RadarTracks& radar_tracks_msg,
                            bool& complete);

    uint32_t    sequence_id_   = 0u;
    std::string frame_id_;
    uint32_t    stamp_tv_sec_  = 0u;
    uint32_t    stamp_tv_nsec_ = 0u;

    rclcpp::Logger logger_;
};

} // namespace au_4d_radar

#endif // MESSAGE_PARSE_HPP
