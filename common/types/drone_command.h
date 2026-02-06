#pragma once

#include <cstdint>

namespace terrain_follower {
namespace ipc {

enum class CommandType : uint8_t {
    NONE = 0,
    ARM = 1,
    DISARM = 2,
    TAKEOFF = 3,
    LAND = 4,
    OFFBOARD = 5,
    SET_POSITION = 6,
    HOLD = 7
};

enum class CommandStatus : uint8_t {
    IDLE = 0,           // No command pending
    PENDING = 1,        // Command received, not yet executed
    EXECUTING = 2,      // Command being executed
    SUCCESS = 3,        // Command completed successfully
    FAILED = 4          // Command failed
};

struct DroneCommand {
    CommandType type;
    CommandStatus status;
    uint64_t timestamp_us;  // When command was issued
    
    // Command parameters
    float param1;  // e.g., takeoff altitude, North position
    float param2;  // East position
    float param3;  // Down position (negative = altitude)
    float param4;  // Yaw angle (radians)
    
    // Result message
    char message[128];
};

} // namespace ipc
} // namespace terrain_follower
