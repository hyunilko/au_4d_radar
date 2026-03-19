/**
 * @file au_4d_radar.cpp
 * @author antonioko@au-sensor.com
 * @brief AU 4D Radar ROS2 driver node
 * @version 2.0
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 *
 */

#include "au_4d_radar.hpp"
#include "util/yamlParser.hpp"

namespace au_4d_radar {

/* ---------- static member definitions ------------------------------------ */
std::atomic<bool>        device_au_radar_node::shutdown_requested_{false};
device_au_radar_node*    device_au_radar_node::instance_ = nullptr;

/* ---------- constructor -------------------------------------------------- */

/**
 * @brief Constructs the AU 4D Radar ROS2 driver node.
 *
 * @details Creates all publishers, initialises YAML settings, then starts the
 *          long-frame and short-frame handlers (which open the PCAN channel).
 *
 * @param options ROS2 node options forwarded to the base Node constructor.
 */
device_au_radar_node::device_au_radar_node(const rclcpp::NodeOptions& options)
    : Node("device_au_radar_node", options)
    , can_fd_transfer_(PcanFdTransfer::Config{})
    , can_short_handler_(this, can_fd_transfer_)
    , can_long_handler_(this, can_fd_transfer_)
    , adm_tf_listener_(this)
{
    instance_ = this;

    /* QoS: reliable sensor data */
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    pub_radar_point_cloud2_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(
            "/device/au/radar/point_cloud2", qos);

    pub_radar_scan_ =
        create_publisher<radar_msgs::msg::RadarScan>(
            "/device/au/radar/scan", qos);

    pub_radar_track_ =
        create_publisher<radar_msgs::msg::RadarTracks>(
            "/device/au/radar/track", qos);

    pub_radar_mon_ =
        create_publisher<mon_msgs::msg::RadarHealth>(
            "/device/au/radar/status", qos);

#ifdef DEBUG_BUILD
    if (rcutils_logging_set_logger_level(
            get_logger().get_name(),
            RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
        RCLCPP_WARN(get_logger(), "Failed to set logger level to DEBUG");
    }
#endif

    initInterruptHandler();
    YamlParser::init();

    /* 콜백 등록 순서:
     *   1. can_long_handler_.start()  → PcanLongFrame::set_rx_callback()  등록
     *   2. can_short_handler_.start() → PcanShortFrame::set_rx_callback() 등록
     *   3. can_fd_transfer_.start_rx() → 수신 스레드 시작
     *
     * Short / Long 콜백이 모두 등록된 뒤 수신 스레드를 시작해야
     * 첫 HEART_BEAT 도착 시 TIME_SYNC 를 정상 응답할 수 있다. */
    can_long_handler_.start();
    can_short_handler_.start();
    can_fd_transfer_.start_rx();   /* 모든 콜백 등록 완료 후 수신 시작 */

    RCLCPP_DEBUG(get_logger(), "AU 4D Radar Driver Node started");
}

/* ---------- signal handling ---------------------------------------------- */

/**
 * @brief POSIX signal handler — sets shutdown_requested_ and calls rclcpp::shutdown().
 *
 * @note Only async-signal-safe operations are performed here.
 *       Actual resource cleanup happens in the destructor.
 *
 * @param sig Signal number received (SIGINT / SIGHUP / SIGTERM).
 */
void device_au_radar_node::interruptHandler(int sig)
{
    (void)sig;
    shutdown_requested_.store(true);
    rclcpp::shutdown();   /* rclcpp::shutdown() is async-signal-safe */
}

/**
 * @brief Registers interruptHandler() for SIGINT, SIGHUP, and SIGTERM.
 *
 * @return 0 always.
 */
int device_au_radar_node::initInterruptHandler(void)
{
    signal(SIGINT,  interruptHandler); // SIGINT is a signal generated when an interrupt (Ctrl+C) is received in the terminal.
    signal(SIGHUP,  interruptHandler); // SIGHUP is a signal that occurs when a terminal connection is disconnected (Hangup) or a control terminal is closed.
    signal(SIGTERM, interruptHandler); // SIGTERM is the default termination signal to terminate a process and is delivered by default in commands such as the kill command.
    return 0;
}

/* ---------- publish helpers ---------------------------------------------- */

/**
 * @brief Thread-safe publish of a RadarScan message on /device/au/radar/scan.
 * @param msg Message to publish.
 */
void device_au_radar_node::publishRadarScanMsg(radar_msgs::msg::RadarScan& msg)
{
    std::lock_guard<std::mutex> lk(mtx_msg_publisher_);
    pub_radar_scan_->publish(msg);
}

/**
 * @brief Thread-safe publish of a RadarTracks message on /device/au/radar/track.
 * @param msg Message to publish.
 */
void device_au_radar_node::publishRadarTrackMsg(radar_msgs::msg::RadarTracks& msg)
{
    std::lock_guard<std::mutex> lk(mtx_msg_publisher_);
    pub_radar_track_->publish(msg);
}

/**
 * @brief Thread-safe publish of a PointCloud2 message on /device/au/radar/point_cloud2.
 * @param msg Message to publish.
 */
void device_au_radar_node::publishRadarPointCloud2(sensor_msgs::msg::PointCloud2& msg)
{
    std::lock_guard<std::mutex> lk(mtx_msg_publisher_);
    pub_radar_point_cloud2_->publish(msg);
}

/**
 * @brief Thread-safe publish of a RadarHealth message on /device/au/radar/status.
 * @param msg Message to publish.
 */
void device_au_radar_node::publishHeartbeat(mon_msgs::msg::RadarHealth& msg)
{
    std::lock_guard<std::mutex> lk(mtx_msg_publisher_);
    pub_radar_mon_->publish(msg);
}

} // namespace au_4d_radar

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(au_4d_radar::device_au_radar_node)
