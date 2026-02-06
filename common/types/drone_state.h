#pragma once

#include <cstdint>
#include <ctime>

namespace terrain_follower {
namespace ipc {

struct DroneState {
    // Timestamp
    uint64_t timestamp_us;  // Microseconds since epoch
    
    // Position (NED frame - North, East, Down in meters)
    double position_n;
    double position_e;
    double position_d;
    
    // Velocity (NED frame in m/s)
    float velocity_n;
    float velocity_e;
    float velocity_d;
    
    // Attitude (Euler angles in radians)
    float roll;
    float pitch;
    float yaw;
    
    // Angular velocity (rad/s)
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    
    // Battery
    float battery_voltage;      // Volts
    float battery_remaining;    // Percentage (0-100)
    
    // Status
    bool armed;
    bool in_air;
    uint8_t flight_mode;  // PX4 flight mode
    
    // GPS
    double latitude;   // Degrees
    double longitude;  // Degrees
    float altitude;    // Meters above MSL
    uint8_t gps_fix;   // 0: no fix, 3: 3D fix
    uint8_t num_satellites;
    
    // Health flags
    bool is_connected;
    bool telemetry_valid;
};

} // namespace ipc
} // namespace terrain_follower
