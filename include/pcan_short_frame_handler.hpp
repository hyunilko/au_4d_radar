#pragma once

#include <cstdint>
#include <vector>

#include "rclcpp/logger.hpp"
#include "pcan_short_frame.hpp"

namespace au_4d_radar
{
    class device_au_radar_node;

    class PcanShortFrameHandler
    {
    public:
        explicit PcanShortFrameHandler(
            device_au_radar_node* node,
            PcanShortFrame& can,
            rclcpp::Logger logger = rclcpp::get_logger("PcanShortFrameHandler"),
            bool quiet = false);

        void start(void);
        void stop(void);

    private:
        void handle_short_frame(uint8_t dev_id, ShortCanCmd cmd, uint32_t uniq_id,
                                const std::vector<uint8_t>& data);

        bool send_time_sync(uint8_t dev_id, uint32_t uniq_id);

        PcanShortFrame& can_short_;
        rclcpp::Logger  logger_;
        bool            quiet_ = false;

        device_au_radar_node* radar_node_;
    };

} // namespace au_4d_radar
