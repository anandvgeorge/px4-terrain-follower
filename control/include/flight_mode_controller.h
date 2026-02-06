#pragma once

#include <memory>
#include <chrono>
#include "ipc/shared_memory.h"
#include "ipc/command_channel.h"
#include "types/drone_state.h"
#include "types/drone_command.h"

namespace terrain_follower {
namespace control {

/**
 * @brief Unified flight mode controller for PX4 drone
 * 
 * Provides interface for:
 * - Basic flight modes (ARM, TAKEOFF, LAND, HOLD)
 * - Offboard position control with setpoints
 * - Offboard velocity control with setpoints
 * - State queries
 */
class FlightModeController {
public:
    FlightModeController();
    ~FlightModeController();
    
    // ============================================
    // INITIALIZATION
    // ============================================
    bool init();
    
    // ============================================
    // BASIC FLIGHT MODES (PX4 native)
    // ============================================
    bool arm();
    bool disarm();
    bool takeoff(float altitude_m = 5.0f);
    bool land();
    bool hold();  // PX4 HOLD mode
    
    // ============================================
    // OFFBOARD MODE - POSITION CONTROL
    // ============================================
    
    /**
     * @brief Start offboard mode with position control
     * Automatically holds current position if no setpoint sent
     */
    bool start_offboard_position();
    
    /**
     * @brief Set absolute position setpoint (NED frame)
     * @param north North position (m)
     * @param east East position (m)
     * @param down Down position (m, negative = altitude)
     * @param yaw Yaw angle (radians)
     */
    bool set_position_offboard(float north, float east, float down, float yaw);
    
    /**
     * @brief Hover at current position
     * Sends current position as setpoint
     */
    bool hover_position_offboard();
    
    // ============================================
    // OFFBOARD MODE - VELOCITY CONTROL
    // ============================================
    
    /**
     * @brief Start offboard mode with velocity control
     * Automatically hovers (zero velocity) if no setpoint sent
     */
    bool start_offboard_velocity();
    
    /**
     * @brief Set velocity setpoint (NED frame)
     * @param vn North velocity (m/s)
     * @param ve East velocity (m/s)
     * @param vd Down velocity (m/s, negative = climb)
     * @param yaw_rate Yaw rate (rad/s)
     */
    bool set_velocity_offboard(float vn, float ve, float vd, float yaw_rate);
    
    /**
     * @brief Hover in place (zero velocity)
     */
    bool hover_velocity_offboard();
    
    // ============================================
    // OFFBOARD MODE - EXIT
    // ============================================
    
    /**
     * @brief Stop offboard mode
     */
    bool stop_offboard();
    
    // ============================================
    // STATE QUERIES
    // ============================================
    bool is_armed();
    bool is_in_air();
    bool is_connected();
    bool is_ready_to_arm();
    
    // ============================================
    // GET CURRENT STATE
    // ============================================
    float get_altitude();
    bool get_position(float& north, float& east, float& down, float& yaw);
    bool get_velocity(float& vn, float& ve, float& vd);
    
private:
    enum class OffboardMode {
        NONE,      // Not in offboard
        POSITION,  // Position control
        VELOCITY   // Velocity control
    };
    
    bool send_command(ipc::CommandType type, float param1 = 0.0f, 
                     float param2 = 0.0f, float param3 = 0.0f, float param4 = 0.0f,
                     int timeout_ms = 5000);
    
    std::unique_ptr<ipc::SharedMemory<ipc::DroneState>> state_shm_;
    std::unique_ptr<ipc::CommandChannel> cmd_channel_;
    
    OffboardMode current_offboard_mode_{OffboardMode::NONE};
};

} // namespace control
} // namespace terrain_follower
