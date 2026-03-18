/**
 * @file au_4d_radar.cpp
 * @author antonioko@au-sensor.com
 * @brief
 * @version 1.1
 * @date 2024-09-11
 *
 * @copyright Copyright AU (c) 2026
 *
 */

#include <unistd.h>

#include "au_4d_radar.hpp"
#include "util/yamlParser.hpp"

#define PUB_TIME 10ms

namespace au_4d_radar {

namespace {
PcanFdTransfer::Config make_pcan_fd_transfer_config()
{
    return PcanFdTransfer::Config{};
}

PcanShortFrameConfig make_pcan_short_frame_config()
{
    return PcanShortFrameConfig{};
}

PcanLongFrameConfig make_pcan_long_frame_config()
{
    return PcanLongFrameConfig{};
}
} // namespace

device_au_radar_node* device_au_radar_node::instance_ = nullptr;

/**
 * @brief Constructs the AU 4D Radar ROS2 node.
 *
 * @details Creates all ROS2 publishers, initialises YAML settings, starts the short-frame
 *          and long-frame handlers, and launches the PCAN RX dispatch thread.
 *
 * @param options ROS2 node options forwarded to the base Node constructor.
 */
device_au_radar_node::device_au_radar_node(const rclcpp::NodeOptions & options)
: Node("device_au_radar_node", options),
  can_fd_transfer_(make_pcan_fd_transfer_config(),
                   make_pcan_short_frame_config(),
                   make_pcan_long_frame_config()),
  can_short_handler_(this, can_fd_transfer_),
  can_long_handler_(this, can_fd_transfer_),
  adm_tf_listener_(this)
{
    instance_ = this;

    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    pub_radar_point_cloud2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/device/au/radar/point_cloud2", qos);

    pub_radar_scan = this->create_publisher<radar_msgs::msg::RadarScan>(
                    "/device/au/radar/scan",
                    qos);

    pub_radar_track = this->create_publisher<radar_msgs::msg::RadarTracks>(
                    "/device/au/radar/track",
                    qos);

    pub_radar_mon = this->create_publisher<mon_msgs::msg::RadarHealth>(
                    "/device/au/radar/status",
                    qos);

#ifdef DEBUG_BUILD
  if (rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set logger level to DEBUG");
  }
#endif

    initInterruptHandler();
    YamlParser::init();
    can_long_handler_.start();
    can_short_handler_.start();
    startPcanRxDispatch();

    RCLCPP_DEBUG(this->get_logger(), "Start AU 4D Radar Driver Node");
}

/**
 * @brief Destructor. Stops the PCAN RX dispatch thread and both frame handlers.
 */
device_au_radar_node::~device_au_radar_node()
{
    stopPcanRxDispatch();
    can_long_handler_.stop();
    can_short_handler_.stop();
}

/**
 * @brief Starts the PCAN RX dispatch thread if it is not already running.
 *
 * @details Uses an atomic exchange to guarantee at most one dispatch thread is active.
 */
void device_au_radar_node::startPcanRxDispatch(void)
{
    if (pcan_rx_running_.exchange(true)) {
        return;
    }

    pcan_rx_thread_ = std::thread(&device_au_radar_node::pcanRxDispatchLoop, this);
}

/**
 * @brief Signals the PCAN RX dispatch thread to exit and joins it.
 */
void device_au_radar_node::stopPcanRxDispatch(void)
{
    pcan_rx_running_.store(false);
    if (pcan_rx_thread_.joinable()) {
        pcan_rx_thread_.join();
    }
}

/**
 * @brief Main CAN RX loop: reads frames from hardware and dispatches to handlers.
 *
 * @details Runs in a dedicated thread. Each iteration calls can_fd_transfer_.read_frame().
 *          Empty-queue results trigger a 1 ms sleep; data frames are offered first to the
 *          short-frame handler, then to the long-frame handler. Unrecognised CAN IDs are
 *          logged as warnings.
 */
void device_au_radar_node::pcanRxDispatchLoop(void)
{
    while (pcan_rx_running_.load()) {
        PcanFdTransfer::RxFrame frame{};
        const PcanFdTransfer::ReadStatus st = can_fd_transfer_.read_frame(frame);

        if (st == PcanFdTransfer::ReadStatus::Empty) {
            usleep(1000);
            continue;
        }

        if (st != PcanFdTransfer::ReadStatus::Ok) {
            continue;
        }

        if (can_short_handler_.handle_can_frame(frame.can_id, frame.data, frame.data_len)) {
            continue;
        }

        if (can_long_handler_.handle_can_frame(frame.can_id, frame.data, frame.data_len)) {
            continue;
        }

        RCLCPP_WARN(this->get_logger(),
                    "Unknown CAN ID: 0x%03X len=%u",
                    frame.can_id,
                    frame.data_len);
    }
}

/**
 * @brief POSIX signal handler that performs a clean shutdown on fatal signals.
 *
 * @details Stops the RX dispatch thread and both frame handlers, then calls exit(0).
 *          Registered for SIGINT, SIGHUP, SIGKILL, SIGSEGV, and SIGTERM.
 *
 * @param sig Signal number received.
 */
void device_au_radar_node::interruptHandler(int sig) {
    RCLCPP_ERROR(rclcpp::get_logger("interruptHandler"), "signum=%d", sig);

    if (sig == SIGINT || sig == SIGHUP || sig == SIGKILL || sig == SIGSEGV || sig == SIGTERM) {
        RCLCPP_ERROR(rclcpp::get_logger("radar_node"), "interruptHandler performed");

        instance_->stopPcanRxDispatch();
        instance_->can_long_handler_.stop();
        instance_->can_short_handler_.stop();

        exit(0);
    }
}

/**
 * @brief Declares or retrieves a ROS2 parameter, writing its value into @p variable.
 *
 * @tparam Param  Parameter value type (deduced from @p variable).
 * @param nh       Shared pointer to the node on which to operate.
 * @param name     Parameter name.
 * @param variable Reference updated with the parameter value.
 */
template<typename Param>
void device_au_radar_node::get_param(rclcpp::Node::SharedPtr nh, const std::string& name, Param& variable) {
    using var_type = std::remove_reference_t<decltype(variable)>;

    if (!nh->has_parameter(name)) {
        variable = nh->declare_parameter<var_type>(name, variable);
    } else {
        nh->get_parameter(name, variable);
    }
}

/**
 * @brief Thread-safe publish of a RadarScan message on /device/au/radar/scan.
 *
 * @param radar_scan_msg Message to publish.
 */
void device_au_radar_node::publishRadarScanMsg(radar_msgs::msg::RadarScan &radar_scan_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    pub_radar_scan->publish(radar_scan_msg);
}

/**
 * @brief Thread-safe publish of a RadarTracks message on /device/au/radar/track.
 *
 * @param radar_tracks_msg Message to publish.
 */
void device_au_radar_node::publishRadarTrackMsg(radar_msgs::msg::RadarTracks &radar_tracks_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    pub_radar_track->publish(radar_tracks_msg);
}

/**
 * @brief Thread-safe publish of a PointCloud2 message on /device/au/radar/point_cloud2.
 *
 * @param radar_cloud_msg Message to publish.
 */
void device_au_radar_node::publishRadarPointCloud2(sensor_msgs::msg::PointCloud2& radar_cloud_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    pub_radar_point_cloud2->publish(radar_cloud_msg);
}

/**
 * @brief Thread-safe publish of a RadarHealth message on /device/au/radar/status.
 *
 * @param radar_health_msg Message to publish.
 */
void device_au_radar_node::publishHeartbeat(mon_msgs::msg::RadarHealth& radar_health_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    pub_radar_mon->publish(radar_health_msg);
}

/**
 * @brief Registers interruptHandler() for SIGINT and SIGHUP.
 *
 * @return 0 always.
 */
int device_au_radar_node::initInterruptHandler(void) {
    signal(SIGINT, interruptHandler);
    signal(SIGHUP, interruptHandler);
    return 0;
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(au_4d_radar::device_au_radar_node)
