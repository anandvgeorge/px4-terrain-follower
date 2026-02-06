#include "control_node.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

using namespace terrain_follower;

control::ControlNode::ControlNode() {
    state_shm_ = std::make_unique<ipc::SharedMemory<ipc::DroneState>>("px4_drone_state");
    cmd_channel_ = std::make_unique<ipc::CommandChannel>("px4_drone_command");
}

control::ControlNode::~ControlNode() = default;

bool control::ControlNode::init() {
    std::cout << "Initializing ControlNode..." << std::endl;
    
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

bool control::ControlNode::send_command(ipc::CommandType type, float param1, int timeout_ms) {
    ipc::DroneCommand cmd{};
    cmd.type = type;
    cmd.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    cmd.param1 = param1;
    std::strcpy(cmd.message, "");
    
    // Send command via channel (blocks until response or timeout)
    return cmd_channel_->send_command(cmd, timeout_ms);
}

bool control::ControlNode::arm() {
    std::cout << "Arming..." << std::endl;
    bool success = send_command(ipc::CommandType::ARM);
    if (success) {
        std::cout << "Armed!" << std::endl;
    }
    return success;
}

bool control::ControlNode::takeoff(float altitude_m) {
    std::cout << "Taking off to " << altitude_m << " m..." << std::endl;
    bool success = send_command(ipc::CommandType::TAKEOFF, altitude_m);
    if (success) {
        std::cout << "Takeoff command accepted!" << std::endl;
    }
    return success;
}

bool control::ControlNode::land() {
    std::cout << "Landing..." << std::endl;
    bool success = send_command(ipc::CommandType::LAND);
    if (success) {
        std::cout << "Land command accepted!" << std::endl;
    }
    return success;
}

bool control::ControlNode::disarm() {
    std::cout << "Disarming..." << std::endl;
    bool success = send_command(ipc::CommandType::DISARM);
    if (success) {
        std::cout << "Disarmed!" << std::endl;
    }
    return success;
}

bool control::ControlNode::hold() {
    std::cout << "Activating HOLD mode..." << std::endl;
    bool success = send_command(ipc::CommandType::HOLD);
    if (success) {
        std::cout << "HOLD mode activated!" << std::endl;
    }
    return success;
}

bool control::ControlNode::is_armed() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return state.armed;
    }
    return false;
}

bool control::ControlNode::is_in_air() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return state.in_air;
    }
    return false;
}

bool control::ControlNode::is_ready_to_arm() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return state.is_connected && state.telemetry_valid && !state.armed;
    }
    return false;
}

bool control::ControlNode::is_connected() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return state.is_connected && state.telemetry_valid;
    }
    return false;
}

float control::ControlNode::get_altitude() {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        return -state.position_d;  // NED frame: down is negative
    }
    return 0.0f;
}
