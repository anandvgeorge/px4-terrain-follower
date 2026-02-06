#include "offboard_mode.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace terrain_follower;

control::OffboardMode::OffboardMode() {
    state_shm_ = std::make_unique<ipc::SharedMemory<ipc::DroneState>>("px4_drone_state");
    cmd_shm_ = std::make_unique<ipc::SharedMemory<ipc::DroneCommand>>("px4_drone_command");
}

control::OffboardMode::~OffboardMode() = default;

bool control::OffboardMode::init() {
    if (!state_shm_->open()) {
        std::cerr << "Failed to open state shared memory" << std::endl;
        return false;
    }
    
    if (!cmd_shm_->open()) {
        std::cerr << "Failed to open command shared memory" << std::endl;
        return false;
    }
    
    return true;
}

bool control::OffboardMode::enter_offboard_mode() {
    std::cout << "Entering offboard mode..." << std::endl;
    
    // Send OFFBOARD command to px4_mavlink_interface
    ipc::DroneCommand cmd{};
    cmd.type = ipc::CommandType::OFFBOARD;
    cmd.status = ipc::CommandStatus::PENDING;
    cmd.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    
    cmd_shm_->write(cmd);
    
    // Wait for command completion (with timeout)
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
        if (cmd_shm_->read(cmd)) {
            if (cmd.status == ipc::CommandStatus::SUCCESS) {
                std::cout << "Offboard mode activated successfully" << std::endl;
                return true;
            } else if (cmd.status == ipc::CommandStatus::FAILED) {
                std::cerr << "Offboard mode failed: " << cmd.message << std::endl;
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cerr << "Offboard mode command timed out" << std::endl;
    return false;
}

bool control::OffboardMode::hold_position() {
    // Get current position
    float north, east, down;
    if (!get_current_position(north, east, down)) {
        std::cerr << "Failed to get current position" << std::endl;
        return false;
    }
    
    // Get current yaw
    ipc::DroneState state;
    if (!state_shm_->read(state)) {
        std::cerr << "Failed to read drone state" << std::endl;
        return false;
    }
    
    std::cout << "Holding position at N:" << north << " E:" << east 
              << " D:" << down << " (Alt:" << -down << "m)" << std::endl;
    
    // Send position setpoint command
    ipc::DroneCommand cmd{};
    cmd.type = ipc::CommandType::SET_POSITION;
    cmd.status = ipc::CommandStatus::PENDING;
    cmd.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    cmd.param1 = north;
    cmd.param2 = east;
    cmd.param3 = down;
    cmd.param4 = state.yaw;
    
    cmd_shm_->write(cmd);
    
    // Wait briefly for command to be acknowledged
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    return true;
}

bool control::OffboardMode::get_current_position(float& north, float& east, float& down) {
    ipc::DroneState state;
    if (state_shm_->read(state)) {
        north = state.position_n;
        east = state.position_e;
        down = state.position_d;
        return true;
    }
    return false;
}
