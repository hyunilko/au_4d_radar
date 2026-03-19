/**
 * @file message_parse.cpp
 * @author antonioko@au-sensor.com
 * @brief Implementation of MessageParser — radar packet decode.
 * @version 2.0
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 *
 */

#include <cstdint>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "au_4d_radar.hpp"
#include "util/uuid_helper.hpp"
#include "util/conversion.hpp"

/* -----------------------------------------------------------------------
 * Wire packet header layout (matches S32 firmware).
 * ----------------------------------------------------------------------- */
struct TsPacketHeader
{
    uint32_t uid;       /* unique device ID (LE) */
    uint32_t ts_sec;    /* timestamp seconds (LE) */
    uint32_t ts_nsec;   /* timestamp nanoseconds (LE) */
    uint32_t frame_num;
    float    cycle_time;
    uint32_t total_points;
    uint32_t point_num;
    uint16_t total_pkts;
    uint16_t pkt_num;
};

static constexpr size_t   POINT_STEP_SIZE      = 20u;
static constexpr uint32_t MAX_POINTS_PER_PKT   = 60u;
static constexpr uint32_t MAX_TOTAL_POINTS     = 1600u;
static constexpr uint16_t MAX_PKTS             = 28u;

namespace au_4d_radar
{

/* ----------------------------------------------------------------------- */
/**
 * @brief Deserialises a TsPacketHeader from a little-endian byte buffer.
 *
 * @param p       Pointer to the start of the header in the buffer.
 * @param idx_out Set to the number of bytes consumed (sizeof TsPacketHeader = 32).
 * @return Populated TsPacketHeader.
 */
static TsPacketHeader parse_header(const uint8_t* p, uint32_t& idx_out)
{
    TsPacketHeader h{};
    uint32_t i = 0u;
    h.uid          = Conversion::le_to_u32(&p[i]); i += 4;
    h.ts_sec       = Conversion::le_to_u32(&p[i]); i += 4;
    h.ts_nsec      = Conversion::le_to_u32(&p[i]); i += 4;
    h.frame_num    = Conversion::le_to_u32(&p[i]); i += 4;
    h.cycle_time   = Conversion::convertToFloat(&p[i]); i += 4;
    h.total_points  = Conversion::le_to_u32(&p[i]); i += 4;
    h.point_num    = Conversion::le_to_u32(&p[i]); i += 4;
    h.total_pkts   = Conversion::le_to_u16(&p[i]); i += 2;
    h.pkt_num      = Conversion::le_to_u16(&p[i]); i += 2;
    idx_out = i;
    return h;
}

/**
 * @brief Sanity-checks packet header field ranges and logs an error on failure.
 *
 * @param h        Header to validate.
 * @param frame_id Frame ID string used in the error log message.
 * @param logger   ROS2 logger for error output.
 * @return true if all fields are within valid bounds, false otherwise.
 */
static bool validate_header(const TsPacketHeader& h,
                             const std::string& frame_id,
                             const rclcpp::Logger& logger)
{
    if (h.point_num   > MAX_POINTS_PER_PKT ||
        h.total_points > MAX_TOTAL_POINTS   ||
        h.total_pkts > MAX_PKTS          ||
        h.pkt_num    > MAX_PKTS)
    {
        RCLCPP_ERROR(logger,
                     "Header sanity check failed: radar=%s FN=%u TPN=%u PN=%u "
                     "TPCKN=%u PCKN=%u",
                     frame_id.c_str(),
                     h.frame_num, h.total_points, h.point_num,
                     h.total_pkts, h.pkt_num);
        return false;
    }
    return true;
}

/**
 * @brief Formats a 32-bit UID as an 8-character lowercase hex string.
 *
 * @param uid Sensor unique identifier.
 * @return Hex string, e.g. "0a1b2c3d".
 */
static std::string uid_to_hex(uint32_t uid)
{
    std::ostringstream ss;
    ss << std::hex << std::setw(8) << std::setfill('0') << uid;
    return ss.str();
}

/* ----------------------------------------------------------------------- */

/**
 * @brief Parses one radar scan packet and appends point data to a PointCloud2 message.
 *
 * @details Reads the TsPacketHeader, resolves the sensor pose from YAML, converts
 *          each point from spherical (range/azimuth/elevation) to Cartesian using the
 *          radar's rotation matrix, and appends x/y/z/intensity/velocity float data.
 *          Initialises cloud_msg header and field descriptors on pkt_num == 1.
 *
 * @param p_buff    Pointer to the raw payload buffer starting at the UID field.
 * @param cloud_msg PointCloud2 being assembled; data is appended across packets.
 * @param complete  Set to true when the last packet (total_pkts == pkt_num) is parsed.
 */
void MessageParser::makeRadarPointCloud2Msg(uint8_t* p_buff,
                                             sensor_msgs::msg::PointCloud2& cloud_msg,
                                             bool& complete)
{
    uint32_t idx = 0u;
    const TsPacketHeader header = parse_header(p_buff, idx);

    RadarInfo radar_info = YamlParser::getRadarInfo(header.uid);
    frame_id_ = radar_info.frame_id.empty()
                ? uid_to_hex(header.uid)
                : radar_info.frame_id;

    if (!validate_header(header, frame_id_, logger_)) {
        return;
    }

    complete      = (header.total_pkts == header.pkt_num);
    stamp_tv_sec_  = header.ts_sec;
    stamp_tv_nsec_ = header.ts_nsec;

    if (header.pkt_num == 1u) {
        cloud_msg.header.frame_id      = frame_id_;
        cloud_msg.header.stamp.sec     = stamp_tv_sec_;
        cloud_msg.header.stamp.nanosec = stamp_tv_nsec_;
        cloud_msg.height               = 1u;
        cloud_msg.width                = header.total_points;
        cloud_msg.is_bigendian         = false;
        cloud_msg.point_step           = POINT_STEP_SIZE;
        cloud_msg.row_step             = POINT_STEP_SIZE * header.total_points;
        cloud_msg.is_dense             = true;

        cloud_msg.fields.resize(5u);
        auto set_field = [&](size_t i, const char* name, uint32_t offset) {
            cloud_msg.fields[i].name     = name;
            cloud_msg.fields[i].offset   = offset;
            cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud_msg.fields[i].count    = 1u;
        };
        set_field(0, "x",        0u);
        set_field(1, "y",        4u);
        set_field(2, "z",        8u);
        set_field(3, "intensity", 12u);
        set_field(4, "velocity",  16u);

        cloud_msg.data.clear();
    }

    /* Build rotation matrix (ZYX intrinsic) */
    const Eigen::Matrix3f R =
        (Eigen::AngleAxisf(radar_info.yaw,   Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(radar_info.pitch, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(radar_info.roll,  Eigen::Vector3f::UnitX())).toRotationMatrix();

    static constexpr double kDeg2Rad = M_PI / 180.0;

    for (uint32_t i = 0u; i < header.point_num; ++i) {
        idx += 4u;  /* skip index field */
        const float range     = Conversion::convertToFloat(&p_buff[idx]); idx += 4;
        const float velocity  = Conversion::convertToFloat(&p_buff[idx]); idx += 4;
        const float azimuth   = Conversion::convertToFloat(&p_buff[idx]); idx += 4;
        const float elevation = Conversion::convertToFloat(&p_buff[idx]); idx += 4;
        const float amplitude = Conversion::convertToFloat(&p_buff[idx]); idx += 4;

        const float cos_el = std::cos(static_cast<float>(elevation * kDeg2Rad));
        const float sin_el = std::sin(static_cast<float>(elevation * kDeg2Rad));
        const float cos_az = std::cos(static_cast<float>(azimuth * kDeg2Rad));
        const float sin_az = std::sin(static_cast<float>(azimuth * kDeg2Rad));

        const Eigen::Vector3f local(range * cos_el * sin_az,
                                    range * cos_el * cos_az,
                                    range * sin_el);
        const Eigen::Vector3f world = R * local;

        const float px = world.x() + radar_info.x;
        const float py = world.y() + radar_info.y;
        const float pz = world.z() + radar_info.z;

        uint8_t pt[POINT_STEP_SIZE];
        std::memcpy(pt +  0, &px,        sizeof(float));
        std::memcpy(pt +  4, &py,        sizeof(float));
        std::memcpy(pt +  8, &pz,        sizeof(float));
        std::memcpy(pt + 12, &amplitude, sizeof(float));
        std::memcpy(pt + 16, &velocity,  sizeof(float));
        cloud_msg.data.insert(cloud_msg.data.end(), pt, pt + POINT_STEP_SIZE);
    }

    if (complete) {
        if (cloud_msg.point_step == 0u) {
            RCLCPP_WARN(logger_,
                        "point_step is 0, skipping incomplete frame (mid-stream start)");
            complete = false;
            return;
        }
        cloud_msg.width    = static_cast<uint32_t>(cloud_msg.data.size() / cloud_msg.point_step);
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    }
}

/* ----------------------------------------------------------------------- */

/**
 * @brief Parses one radar scan packet and appends RadarReturn entries to a RadarScan message.
 *
 * @param p_buff         Pointer to the raw payload buffer starting at the UID field.
 * @param radar_scan_msg RadarScan being assembled; returns are appended across packets.
 * @param complete       Set to true when the last packet (total_pkts == pkt_num) is parsed.
 */
void MessageParser::makeRadarScanMsg(uint8_t* p_buff,
                                      radar_msgs::msg::RadarScan& radar_scan_msg,
                                      bool& complete)
{
    uint32_t idx = 0u;
    const TsPacketHeader header = parse_header(p_buff, idx);

    RadarInfo radar_info = YamlParser::getRadarInfo(header.uid);
    frame_id_ = radar_info.frame_id.empty()
                ? uid_to_hex(header.uid)
                : radar_info.frame_id;

    if (!validate_header(header, frame_id_, logger_)) {
        return;
    }

    complete      = (header.total_pkts == header.pkt_num);
    stamp_tv_sec_  = header.ts_sec;
    stamp_tv_nsec_ = header.ts_nsec;

    radar_scan_msg.header.frame_id      = frame_id_;
    radar_scan_msg.header.stamp.sec     = stamp_tv_sec_;
    radar_scan_msg.header.stamp.nanosec = stamp_tv_nsec_;

    for (uint32_t i = 0u; i < header.point_num; ++i) {
        radar_msgs::msg::RadarReturn ret{};
        idx += 4u;  /* skip index field */
        ret.range           = Conversion::convertToFloat(&p_buff[idx]); idx += 4;
        ret.doppler_velocity = Conversion::convertToFloat(&p_buff[idx]); idx += 4;
        ret.azimuth         = Conversion::convertToFloat(&p_buff[idx]); idx += 4;
        ret.elevation       = Conversion::convertToFloat(&p_buff[idx]); idx += 4;
        ret.amplitude       = Conversion::convertToFloat(&p_buff[idx]); idx += 4;
        radar_scan_msg.returns.push_back(ret);
    }
}

/* ----------------------------------------------------------------------- */

/**
 * @brief Stub for radar track message parsing (not yet implemented).
 *
 * @param p_buff           Unused — track wire format not yet defined.
 * @param radar_tracks_msg Unused — no data is populated.
 * @param complete         Unused — always remains false.
 */
void MessageParser::makeRadarTracksMsg(uint8_t* /*p_buff*/,
                                        radar_msgs::msg::RadarTracks& /*radar_tracks_msg*/,
                                        bool& /*complete*/)
{
    /* TODO: implement actual track parsing from firmware wire format */
    RCLCPP_DEBUG(logger_, "makeRadarTracksMsg: not yet implemented");
}

/* ----------------------------------------------------------------------- */

/**
 * @brief Public entry point for PointCloud2 parsing; delegates to makeRadarPointCloud2Msg().
 *
 * @param p_buff   Pointer to the raw payload buffer.
 * @param cloud_msg PointCloud2 message to assemble.
 * @param complete Set to true when the complete frame is parsed.
 */
void MessageParser::parsePointCloud2Msg(uint8_t* p_buff,
                                         sensor_msgs::msg::PointCloud2& cloud_msg,
                                         bool& complete)
{
    makeRadarPointCloud2Msg(p_buff, cloud_msg, complete);
}

/**
 * @brief Public entry point for RadarScan parsing; delegates to makeRadarScanMsg().
 *
 * @param p_buff         Pointer to the raw payload buffer.
 * @param radar_scan_msg RadarScan message to assemble.
 * @param complete       Set to true when the complete frame is parsed.
 */
void MessageParser::parseRadarScanMsg(uint8_t* p_buff,
                                       radar_msgs::msg::RadarScan& radar_scan_msg,
                                       bool& complete)
{
    makeRadarScanMsg(p_buff, radar_scan_msg, complete);
}

/**
 * @brief Public entry point for RadarTracks parsing (stub — delegates to makeRadarTracksMsg()).
 *
 * @param p_buff           Pointer to the raw payload buffer.
 * @param radar_tracks_msg RadarTracks message (not populated in current implementation).
 * @param complete         Completion flag (not set in current implementation).
 */
void MessageParser::parseRadarTrackMsg(uint8_t* p_buff,
                                        radar_msgs::msg::RadarTracks& radar_tracks_msg,
                                        bool& complete)
{
    makeRadarTracksMsg(p_buff, radar_tracks_msg, complete);
}

} // namespace au_4d_radar
