/**
 * @file yamlParser.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief YAML configuration loader — parses system_info.yaml once in init()
 *        and serves all settings from in-memory cache thereafter.
 * @version 1.1
 * @date 2026-03
 *
 * @copyright Copyright AU (c) 2026
 */

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "util/yamlParser.hpp"

/* ---------- static member definitions ------------------------------------ */
std::unordered_map<uint32_t, RadarInfo> YamlParser::radarsMap_;
std::recursive_mutex                    YamlParser::radar_map_mutex_;
YamlParser::AppConfig                   YamlParser::appConfig_;

static constexpr float kDeg2Rad = static_cast<float>(M_PI / 180.0);

/* ---------- private helpers ---------------------------------------------- */

std::string YamlParser::getYamlPath()
{
    return ament_index_cpp::get_package_share_directory("au_4d_radar")
           + "/config/system_info.yaml";
}

/* ---------- init ---------------------------------------------------------- */

void YamlParser::init()
{
    const std::string path = getYamlPath();

    YAML::Node config;
    try {
        config = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("YamlParser"),
                     "Failed to load %s : %s", path.c_str(), e.what());
        return;
    }

    /* --- scalar settings ------------------------------------------------- */
    if (config["POINT_CLOUD2"]) {
        appConfig_.point_cloud2 = config["POINT_CLOUD2"].as<bool>();
    }
    if (config["MESSAGE_NUMBER"]) {
        appConfig_.message_number = config["MESSAGE_NUMBER"].as<uint32_t>();
    }
    if (config["HOSTNAME"]) {
        appConfig_.hostname = config["HOSTNAME"].as<std::string>();
    }

    /* --- radar map -------------------------------------------------------- */
    std::unordered_map<uint32_t, RadarInfo> radarsInfo;

    if (!config["radars"]) {
        RCLCPP_ERROR(rclcpp::get_logger("YamlParser"),
                     "'radars' section not found in %s", path.c_str());
    } else {
        for (const auto& it : config["radars"]) {
            const std::string key = it.first.as<std::string>();

            uint32_t radar_id = 0u;
            std::stringstream ss;
            ss << std::hex << key;
            ss >> radar_id;
            if (ss.fail()) {
                RCLCPP_ERROR(rclcpp::get_logger("YamlParser"),
                             "Cannot parse radar ID: %s", key.c_str());
                continue;
            }

            RadarInfo info;
            if (!it.second["frame_id"]) {
                RCLCPP_ERROR(rclcpp::get_logger("YamlParser"),
                             "'frame_id' missing for radar %s", key.c_str());
                continue;
            }
            info.frame_id = it.second["frame_id"].as<std::string>();

            if (!it.second["xyz"] || !it.second["rpy"]) {
                RCLCPP_ERROR(rclcpp::get_logger("YamlParser"),
                             "'xyz'/'rpy' missing for radar %s", key.c_str());
                continue;
            }

            const YAML::Node xyz = it.second["xyz"];
            const YAML::Node rpy = it.second["rpy"];

            if (xyz.size() != 3u || rpy.size() != 3u) {
                RCLCPP_ERROR(rclcpp::get_logger("YamlParser"),
                             "xyz/rpy must have 3 elements for radar %s", key.c_str());
                continue;
            }

            info.x = xyz[0].as<float>();
            info.y = xyz[1].as<float>();
            info.z = xyz[2].as<float>();

            info.roll  = rpy[0].as<float>() * kDeg2Rad;
            info.pitch = rpy[1].as<float>() * kDeg2Rad;
            info.yaw   = rpy[2].as<float>() * kDeg2Rad;

            RCLCPP_DEBUG(rclcpp::get_logger("YamlParser"),
                         "Loaded radar 0x%08x '%s'  roll=%.4f pitch=%.4f yaw=%.4f",
                         radar_id, info.frame_id.c_str(),
                         info.roll, info.pitch, info.yaw);

            radarsInfo[radar_id] = info;
        }
    }

    std::lock_guard<std::recursive_mutex> lk(radar_map_mutex_);
    radarsMap_ = std::move(radarsInfo);
}

/* ---------- cached scalar getters ---------------------------------------- */

bool YamlParser::getPointCloud2Enabled()
{
    return appConfig_.point_cloud2;
}

uint32_t YamlParser::getMessageNumber()
{
    return appConfig_.message_number;
}

std::string YamlParser::getHostname()
{
    return appConfig_.hostname;
}

/* ---------- radar map accessors ------------------------------------------ */

std::string YamlParser::getFrameIdName(uint32_t radar_id)
{
    std::lock_guard<std::recursive_mutex> lk(radar_map_mutex_);
    const auto it = radarsMap_.find(radar_id);
    return (it != radarsMap_.end()) ? it->second.frame_id : std::string{};
}

bool YamlParser::checkValidFrameId(uint32_t radar_id)
{
    std::lock_guard<std::recursive_mutex> lk(radar_map_mutex_);
    return radarsMap_.count(radar_id) != 0u;
}

RadarInfo YamlParser::getRadarInfo(uint32_t radar_id)
{
    std::lock_guard<std::recursive_mutex> lk(radar_map_mutex_);
    const auto it = radarsMap_.find(radar_id);
    if (it == radarsMap_.end()) {
        RCLCPP_WARN(rclcpp::get_logger("YamlParser"),
                    "radar_id 0x%08x not found", radar_id);
        return RadarInfo{};
    }
    return it->second;
}

RadarInfo YamlParser::getRadarInfo(const std::string& frame_id)
{
    std::lock_guard<std::recursive_mutex> lk(radar_map_mutex_);
    for (const auto& kv : radarsMap_) {
        if (kv.second.frame_id == frame_id) {
            return kv.second;
        }
    }
    RCLCPP_WARN(rclcpp::get_logger("YamlParser"),
                "frame_id '%s' not found", frame_id.c_str());
    return RadarInfo{};
}

void YamlParser::setRadarInfo(const std::string& frame_id, const RadarInfo& radar_info)
{
    std::lock_guard<std::recursive_mutex> lk(radar_map_mutex_);
    for (auto& kv : radarsMap_) {
        if (kv.second.frame_id == frame_id) {
            kv.second = radar_info;
            return;
        }
    }
    RCLCPP_WARN(rclcpp::get_logger("YamlParser"),
                "setRadarInfo: frame_id '%s' not found", frame_id.c_str());
}
