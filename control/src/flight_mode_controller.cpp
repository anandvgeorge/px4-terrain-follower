#include "flight_mode_controller.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

using namespace terrain_follower;

control::FlightModeController::FlightModeController() {
    state_shm_ = std::make_unique<ipc::SharedMemory<ipc::DroneState>>("px4_drone_state");
    cmd_channel_ = std::make_unique<ipc::CommandChannel>("px4_drone_command");
}

control::FlightModeController::~FlightModeController() = default;

bool control::FlightModeController::init() {
    std::cout << "Initializing FlightModeController..." << std::endl;
    
    // Open state shared memory (read-only)
    if (!state_shm_->open()) {
        std::cerr << "Failed to open state shared memory!" << std::endl;
        std::cerr << "Make sure px4_mavlink_interface is running." << std::endl;
        return false;
    }
    
    // Open command channel
    if (!cmd_channel_->open()) {
        std::cerr << "Failed to open command channel!" << std::endl;
        return false;
    }
    
    // Wait for connection
    std::cout << "Waiting for PX4 connection..." << std::endl;
    for (int i = 0; i < 50; i++) {
        if (is_connected()) {
            std::cout << "PX4 connected!" << std::endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cerr << "Timeout waiting for PX4 connection" << std::endl;
    return false;
}

bool control::FlightModeController::send_command(ipc::CommandType type, 
                                                 float param1, float param2, 
                                                 float param3, float param4,
                                                 int timeout_ms) {
    ipc::DroneCommand cmd{};
    cmd.type = type;
    cmd.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    std::strcpy(cmd.message, "");
    
    // Send command via channel (blocks until response or timeout)
    return cmd_channel_->send_command(cmd, timeout_ms);
}

// ============================================
// BASIC FLIGHT MODES
// ============================================

bool control::FlightModeController::arm() {
    std::cout << "Arming..." << std::endl;
    bool success = send_command(ipc::CommandType::ARM);
    if (success) {
        std::cout << "Armed!" << std::endl;
    }
    return success;
}

bool control::FlightModeController::disarm() {
    std::cout << "Disarming..." << std::endl;
    bool success = send_command(ipc::CommandType::DISARM);
    if (success) {
        std::cout << "Disarmed!" << std::endl;
    }
    return success;
}

bool control::FlightModeController::takeoff(float altitude_m) {
    std::cout << "Taking off to " << altitude_m << " m..." << std::endl;
    bool success = send_command(ipc::CommandType::TAKEOFF, altitude_m);
    if (success) {
        std::cout << "Takeoff command accepted!" << std::endl;
    }
    return success;
}

bool control::FlightModeController::land() {
    std::cout << "Landing..." << std::endl;
    bool success = send_command(ipc::CommandType::LAND);
    if (success) {
        std::cout << "Land command accepted!" << std::endl;
        current_offboard_mode_ = OffboardMode::NONE;
    }
    return success;
}

bool control::FlightModeController::hold() {
    std::cout << "Activating HOLD mode..." << std::endl;
    bool success = send_command(ipc::CommandType::HOLD);
    if (success) {
        std::cout << "HOLD mode activated!" << std::endl;
        current_offboard_mode_ = OffboardMode::NONE;
    }
    return success;
}

// ============================================
// OFFBOARD MODE - POSITION CONTROL
// ============================================

bool control::FlightModeController::start_offboard_position() {
    std::cout << "Starting offboard mode (position control)..." << std::endl;
    
    // Get current position
    float north, east, down, yaw;
    if (!get_position(north, east, down, yaw)) {
        std::cerr << "Failed to get current position!" << std::endl;
        return false;
    }
    
    // Send command with current position as initial setpoint
    bool success = send_command(ipc::CommandType::OFFBOARD_POSITION, 
                                north, east, down, yaw);
    if (success) {
        std::cout << "Offboard position mode activated!" << std::endl;
        current_offboard_mode_ = OffboardMode::POSITION;
    }
    return success;
}

bool control::FlightModeController::set_position_offboard(float north, float east, 
                                                          float down, float yaw) {
    if (current_offboard_mode_ != OffboardMode::POSITION) {
        std::cerr << "Not in offboard position mode! Call start_offboard_position() first." 
                  << std::endl;
        return false;
    }
    
    std::cout << "Setting position: N=" << north << " E=" << east 
              << " D=" << down << " (Alt=" << -down << "m) Yaw=" << yaw << std::endl;
    
    return send_command(ipc::CommandType::SET_POSITION, north, east, down, yaw);
}

bool control::FlightModeController::hover_position_offboard() {
    std::cout << "Hovering at current position..." << std::endl;
    
    // Get current position
    float north, east, down, yaw;
    if (!get_position(north, east, down, yaw)) {
        std::cerr << "Failed to get current position!" << std::endl;
        return false;
    }
    
    return set_position_offboard(north, east, down, yaw);
}

// ============================================
// OFFBOARD MODE - VELOCITY CONTROL
// ============================================

bool control::FlightModeController::start_offboard_velocity() {
    std::cout << "Starting offboard mode (velocity control)..." << std::endl;
    
    // Start with zero velocity (hover)
    bool success = send_command(ipc::CommandType::OFFBOARD_VELOCITY, 
                                0.0f, 0.0f, 0.0f, 0.0f);
    if (success) {
        std::cout << "Offboard velocity mode activated!" << std::endl;
        current_offboard_mode_ = OffboardMode::VELOCITY;
    }
    return success;
}

bool control::FlightModeController::set_velocity_offboard(float vn, float ve, 
                                                          float vd, float yaw_rate) {
    if (current_offboard_mode_ != OffboardMode::VELOCITY) {
        std::cerr << "Not in offboard velocity mode! Call start_offboard_velocity() first." 
                  << std::endl;
        return false;
    }
    
    std::cout << "Setting velocity: Vn=" << vn << " Ve=" << ve 
              << " Vd=" << vd << " YawRate=" << yaw_rate << std::endl;
    
    return send_command(ipc::CommandType::SET_VELOCITY, vn, ve, vd, yaw_rate);
}

bool control::FlightModeController::hover_velocity_offboard() {
    std::cout << "Hovering (zero velocity)..." << std::endl;
    return set_velocity_offboard(0.0f, 0.0f, 0.0f, 0.0f);
}

// ============================================
// OFFBOARD MODE - EXIT
// ============================================

bool control::FlightModeController::stop_offboard() {
    std::cout << "Stopping offboard mode..." << std::endl;
    bool success = send_command(ipc::CommandType::STOP_OFFBOARD);
    if (success) {
        std::cout << "Offboard mode stopped!" << std::endl;
        current_offboard_mode_ = OffboardMode::NONE;
    }
    return success;
}

// ============================================
// STATE QUERIES
// ============================================

bool control::FlightModeController::is_armed() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return state.armed;
    }
    return false;
}

bool control::FlightModeController::is_in_air() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return state.in_air;
    }
    return false;
}

bool control::FlightModeController::is_ready_to_arm() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return state.is_connected && state.telemetry_valid && !state.armed;
    }
    return false;
}

bool control::FlightModeController::is_connected() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return state.is_connected && state.telemetry_valid;
    }
    return false;
}

// ============================================
// GET CURRENT STATE
// ============================================

float control::FlightModeController::get_altitude() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return -state.position_d;  // NED frame: down is negative
    }
    return 0.0f;
}

bool control::FlightModeController::get_position(float& north, float& east, 
                                                 float& down, float& yaw) {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        north = state.position_n;
        east = state.position_e;
        down = state.position_d;
        yaw = state.yaw;
        return true;
    }
    return false;
}

bool control::FlightModeController::get_velocity(float& vn, float& ve, float& vd) {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        vn = state.velocity_n;
        ve = state.velocity_e;
        vd = state.velocity_d;
        return true;
    }
    return false;
}
