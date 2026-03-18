/**
 * @file au_4d_radar.cpp
 * @author antonioko@au-sensor.com
 * @brief
 * @version 1.1
 * @date 2024-09-11
 *
 * @copyright Copyright AU (c) 2024
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

device_au_radar_node::~device_au_radar_node()
{
    stopPcanRxDispatch();
    can_long_handler_.stop();
    can_short_handler_.stop();
}

void device_au_radar_node::startPcanRxDispatch(void)
{
    if (pcan_rx_running_.exchange(true)) {
        return;
    }

    pcan_rx_thread_ = std::thread(&device_au_radar_node::pcanRxDispatchLoop, this);
}

void device_au_radar_node::stopPcanRxDispatch(void)
{
    pcan_rx_running_.store(false);
    if (pcan_rx_thread_.joinable()) {
        pcan_rx_thread_.join();
    }
}

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

        const bool quiet = can_fd_transfer_.short_frame_config().quiet &&
                           can_fd_transfer_.long_frame_config().quiet;
        if (!quiet) {
            RCLCPP_WARN(this->get_logger(),
                        "Unknown CAN ID: 0x%03X len=%u",
                        frame.can_id,
                        frame.data_len);
        }
    }
}

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

template<typename Param>
void device_au_radar_node::get_param(rclcpp::Node::SharedPtr nh, const std::string& name, Param& variable) {
    using var_type = std::remove_reference_t<decltype(variable)>;

    if (!nh->has_parameter(name)) {
        variable = nh->declare_parameter<var_type>(name, variable);
    } else {
        nh->get_parameter(name, variable);
    }
}

void device_au_radar_node::publishRadarScanMsg(radar_msgs::msg::RadarScan &radar_scan_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    pub_radar_scan->publish(radar_scan_msg);
}

void device_au_radar_node::publishRadarTrackMsg(radar_msgs::msg::RadarTracks &radar_tracks_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    pub_radar_track->publish(radar_tracks_msg);
}

void device_au_radar_node::publishRadarPointCloud2(sensor_msgs::msg::PointCloud2& radar_cloud_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    pub_radar_point_cloud2->publish(radar_cloud_msg);
}

void device_au_radar_node::publishHeartbeat(mon_msgs::msg::RadarHealth& radar_health_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    pub_radar_mon->publish(radar_health_msg);
}

int device_au_radar_node::initInterruptHandler(void) {
    signal(SIGINT, interruptHandler);
    signal(SIGHUP, interruptHandler);
    return 0;
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(au_4d_radar::device_au_radar_node)
