#ifndef COMMAND_DEF_H
#define COMMAND_DEF_H

#include <cstdint>

inline constexpr std::uint32_t CAN_CTR_CMD_LEN = 4U;

/* Command IDs (U32) */
enum class CanCommand : std::uint32_t
{
    RESET               = 0x10110100U,
    SENSOR_START        = 0x10110203U,
    SENSOR_STOP         = 0x10110302U,
    HI                  = 0x10FF04EBU,
    HEART_BEAT          = 0x10AB4842U,
    TIME_SYNC           = 0x10AB5453U,
};

/* Command IDs (U32) */
enum class CanLongCommand : std::uint32_t
{
    MSG_POINT_CLOUD     = 0x506C5043U,
};

#endif /* COMMAND_DEF_H */