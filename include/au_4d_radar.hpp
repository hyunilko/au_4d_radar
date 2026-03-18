#ifndef AU_4D_RADAR_HPP
#define AU_4D_RADAR_HPP

#include <atomic>
#include <csignal>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "mon_msgs/msg/radar_health.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>

#include "pcan_short_frame_handler.hpp"
#include "pcan_long_frame_handler.hpp"
#include "adm_tf_listener.hpp"

namespace au_4d_radar
{

class device_au_radar_node : public rclcpp::Node
{
public:
    explicit device_au_radar_node(const rclcpp::NodeOptions& options);

    /* ----- publish API (called by handlers) ------------------------------ */
    void publishHeartbeat(mon_msgs::msg::RadarHealth& msg);
    void publishRadarPointCloud2(sensor_msgs::msg::PointCloud2& msg);
    void publishRadarScanMsg(radar_msgs::msg::RadarScan& msg);
    void publishRadarTrackMsg(radar_msgs::msg::RadarTracks& msg);

    /* ----- ROS parameter helper ----------------------------------------- */
    template<typename Param>
    void get_param(rclcpp::Node::SharedPtr nh,
                   const std::string& name,
                   Param& variable)
    {
        using T = std::remove_reference_t<decltype(variable)>;
        if (!nh->has_parameter(name)) {
            variable = nh->declare_parameter<T>(name, variable);
        } else {
            nh->get_parameter(name, variable);
        }
    }

private:
    /* ----- signal handling ---------------------------------------------- */
    static void interruptHandler(int sig);
    int         initInterruptHandler();

    /* ----- internal members --------------------------------------------- */

    /* CAN transport — must be constructed before the handlers that reference it */
    PcanFdTransfer         can_fd_transfer_;

    /* Frame handlers hold a reference to can_fd_transfer_ */
    PcanShortFrameHandler  can_short_handler_;
    PcanLongFrameHandler   can_long_handler_;

    /* TF listener for radar pose updates */
    AdmTFListener          adm_tf_listener_;

    /* ----- ROS publishers ----------------------------------------------- */
    rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr    pub_radar_scan_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_radar_point_cloud2_;
    rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr  pub_radar_track_;
    rclcpp::Publisher<mon_msgs::msg::RadarHealth>::SharedPtr    pub_radar_mon_;

    /* ----- thread-safety for publish calls ------------------------------ */
    std::mutex mtx_msg_publisher_;

    /* ----- signal handler support --------------------------------------- */
    /* Set to true in the signal handler (async-signal-safe); the node's
     * destructor checks this flag for graceful cleanup. */
    static std::atomic<bool>          shutdown_requested_;
    static device_au_radar_node*      instance_;
};

} // namespace au_4d_radar

#endif  // AU_4D_RADAR_HPP
