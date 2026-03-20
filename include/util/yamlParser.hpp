#ifndef YAML_PARSER_HPP
#define YAML_PARSER_HPP

#include <string>
#include <unordered_map>
#include <mutex>

struct RadarInfo {
    std::string frame_id;
    float x     = 0.0f;
    float y     = 0.0f;
    float z     = 0.0f;
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
};

class YamlParser {
public:
    /* ---------------------------------------------------------------
     * init() must be called once at startup (e.g. from the ROS2 node
     * constructor) before any other method is used. It parses
     * system_info.yaml once and caches all settings.
     * --------------------------------------------------------------- */
    static void init();

    /* Cached scalar settings — O(1), no file I/O after init() */
    static bool        getPointCloud2Enabled();
    static uint32_t    getMessageNumber();
    static std::string getHostname();

    /* Radar map accessors */
    static std::string getFrameIdName(uint32_t radar_id);
    static bool        checkValidFrameId(uint32_t radar_id);
    static RadarInfo   getRadarInfo(uint32_t radar_id);
    static RadarInfo   getRadarInfo(const std::string& frame_id);
    static void        setRadarInfo(const std::string& frame_id, const RadarInfo& radar_info);

private:
    YamlParser()  = default;
    ~YamlParser() = default;

    static std::string getYamlPath();

    static std::unordered_map<uint32_t, RadarInfo> radarsMap_;
    static std::recursive_mutex                    radar_map_mutex_;

    /* Cached scalar config — populated once in init() */
    struct AppConfig {
        bool        point_cloud2    = false;
        uint32_t    message_number  = 1u;
        std::string hostname;
    };
    static AppConfig appConfig_;
};

#endif /* YAML_PARSER_HPP */
