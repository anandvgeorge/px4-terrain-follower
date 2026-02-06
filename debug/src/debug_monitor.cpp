#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <cmath>

#include "types/drone_state.h"
#include "types/drone_command.h"
#include "ipc/shared_memory.h"

using namespace terrain_follower;

std::atomic<bool> should_exit{false};

void signal_handler(int signum) {
    should_exit = true;
}

enum class DisplayMode {
    STATE,
    COMMAND,
    BOTH
};

std::atomic<DisplayMode> current_mode{DisplayMode::STATE};

void print_help() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  PX4 Debug Monitor - Controls" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  s - Show STATE only" << std::endl;
    std::cout << "  c - Show COMMAND only" << std::endl;
    std::cout << "  b - Show BOTH" << std::endl;
    std::cout << "  h - Show this help" << std::endl;
    std::cout << "  q - Quit" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

void input_thread() {
    while (!should_exit) {
        char c;
        std::cin >> c;
        switch(c) {
            case 's':
            case 'S':
                current_mode = DisplayMode::STATE;
                std::cout << "\n>>> Switched to STATE view <<<\n" << std::endl;
                break;
            case 'c':
            case 'C':
                current_mode = DisplayMode::COMMAND;
                std::cout << "\n>>> Switched to COMMAND view <<<\n" << std::endl;
                break;
            case 'b':
            case 'B':
                current_mode = DisplayMode::BOTH;
                std::cout << "\n>>> Switched to BOTH view <<<\n" << std::endl;
                break;
            case 'h':
            case 'H':
                print_help();
                break;
            case 'q':
            case 'Q':
                should_exit = true;
                break;
        }
    }
}

const char* command_type_to_string(ipc::CommandType type) {
    switch(type) {
        case ipc::CommandType::NONE: return "NONE";
        case ipc::CommandType::ARM: return "ARM";
        case ipc::CommandType::DISARM: return "DISARM";
        case ipc::CommandType::TAKEOFF: return "TAKEOFF";
        case ipc::CommandType::LAND: return "LAND";
        case ipc::CommandType::OFFBOARD_POSITION: return "OFFBOARD_POSITION";
        case ipc::CommandType::SET_POSITION: return "SET_POSITION";
        case ipc::CommandType::HOLD: return "HOLD";
        case ipc::CommandType::OFFBOARD_VELOCITY: return "OFFBOARD_VELOCITY";
        case ipc::CommandType::SET_VELOCITY: return "SET_VELOCITY";
        case ipc::CommandType::STOP_OFFBOARD: return "STOP_OFFBOARD";
        default: return "UNKNOWN";
    }
}

const char* command_status_to_string(ipc::CommandStatus status) {
    switch(status) {
        case ipc::CommandStatus::IDLE: return "IDLE";
        case ipc::CommandStatus::PENDING: return "PENDING";
        case ipc::CommandStatus::EXECUTING: return "EXECUTING";
        case ipc::CommandStatus::SUCCESS: return "SUCCESS";
        case ipc::CommandStatus::FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

void display_state(const ipc::DroneState& state) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "         DRONE STATE" << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::cout << "\n[CONNECTION]" << std::endl;
    std::cout << "  Connected: " << (state.is_connected ? "YES" : "NO") << std::endl;
    std::cout << "  Valid:     " << (state.telemetry_valid ? "YES" : "NO") << std::endl;
    
    if (!state.telemetry_valid) {
        std::cout << "\nWaiting for telemetry..." << std::endl;
        return;
    }
    
    std::cout << "\n[POSITION NED]" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  N: " << std::setw(7) << state.position_n << " m  ";
    std::cout << "E: " << std::setw(7) << state.position_e << " m  ";
    std::cout << "D: " << std::setw(7) << state.position_d << " m" << std::endl;
    
    std::cout << "\n[VELOCITY NED]" << std::endl;
    std::cout << "  N: " << std::setw(6) << state.velocity_n << " m/s  ";
    std::cout << "E: " << std::setw(6) << state.velocity_e << " m/s  ";
    std::cout << "D: " << std::setw(6) << state.velocity_d << " m/s" << std::endl;
    
    std::cout << "\n[ATTITUDE]" << std::endl;
    std::cout << "  Roll:  " << std::setw(6) << (state.roll * 180.0f / M_PI) << " deg  ";
    std::cout << "Pitch: " << std::setw(6) << (state.pitch * 180.0f / M_PI) << " deg  ";
    std::cout << "Yaw:   " << std::setw(6) << (state.yaw * 180.0f / M_PI) << " deg" << std::endl;
    
    std::cout << "\n[STATUS]" << std::endl;
    std::cout << "  Armed: " << (state.armed ? "YES" : "NO") << "  ";
    std::cout << "In Air: " << (state.in_air ? "YES" : "NO") << "  ";
    std::cout << "Flight Mode: " << (int)state.flight_mode << std::endl;
    
    std::cout << "\n[BATTERY]" << std::endl;
    std::cout << std::setprecision(1);
    std::cout << "  Voltage: " << state.battery_voltage << " V  ";
    std::cout << "Remaining: " << state.battery_remaining << " %" << std::endl;
}

void display_command(const ipc::DroneCommand& cmd) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "         COMMAND STATE" << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::cout << "\n[COMMAND]" << std::endl;
    std::cout << "  Type:   " << command_type_to_string(cmd.type) 
              << " (" << static_cast<int>(cmd.type) << ")" << std::endl;
    std::cout << "  Status: " << command_status_to_string(cmd.status)
              << " (" << static_cast<int>(cmd.status) << ")" << std::endl;
    
    std::cout << "\n[PARAMETERS]" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Param1: " << cmd.param1 << std::endl;
    std::cout << "  Param2: " << cmd.param2 << std::endl;
    std::cout << "  Param3: " << cmd.param3 << std::endl;
    std::cout << "  Param4: " << cmd.param4 << std::endl;
    
    std::cout << "\n[METADATA]" << std::endl;
    std::cout << "  Timestamp: " << cmd.timestamp_us << " us" << std::endl;
    std::cout << "  Message:   " << (cmd.message[0] ? cmd.message : "<empty>") << std::endl;
    
    // Interpret parameters based on command type
    if (cmd.type == ipc::CommandType::TAKEOFF) {
        std::cout << "\n[TAKEOFF Details]" << std::endl;
        std::cout << "  Target Altitude: " << cmd.param1 << " m" << std::endl;
    } else if (cmd.type == ipc::CommandType::SET_POSITION) {
        std::cout << "\n[SET_POSITION Details]" << std::endl;
        std::cout << "  North: " << cmd.param1 << " m" << std::endl;
        std::cout << "  East:  " << cmd.param2 << " m" << std::endl;
        std::cout << "  Down:  " << cmd.param3 << " m (Alt: " << -cmd.param3 << " m)" << std::endl;
        std::cout << "  Yaw:   " << cmd.param4 << " rad (" 
                  << (cmd.param4 * 180.0 / M_PI) << " deg)" << std::endl;
    }
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "========================================" << std::endl;
    std::cout << "    PX4 Debug Monitor v1.0" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Open shared memories
    ipc::SharedMemory<ipc::DroneState> state_shm("px4_drone_state");
    ipc::SharedMemory<ipc::DroneCommand> cmd_shm("px4_drone_command");
    
    std::cout << "\nConnecting to shared memory..." << std::endl;
    
    bool state_ok = state_shm.open();
    bool cmd_ok = cmd_shm.open();
    
    if (state_ok) {
        std::cout << "  ✓ Connected to /px4_drone_state" << std::endl;
    } else {
        std::cout << "  ✗ Failed to open /px4_drone_state" << std::endl;
    }
    
    if (cmd_ok) {
        std::cout << "  ✓ Connected to /px4_drone_command" << std::endl;
    } else {
        std::cout << "  ✗ Failed to open /px4_drone_command" << std::endl;
    }
    
    if (!state_ok && !cmd_ok) {
        std::cerr << "\nError: Could not connect to any shared memory!" << std::endl;
        std::cerr << "Make sure px4_mavlink_interface is running." << std::endl;
        return 1;
    }
    
    print_help();
    
    // Start input thread
    std::thread input(input_thread);
    input.detach();
    
    ipc::DroneState state;
    ipc::DroneCommand cmd;
    
    while (!should_exit) {
        DisplayMode mode = current_mode.load();
        
        // Clear screen (optional)
        // std::cout << "\033[2J\033[1;1H";
        
        if (mode == DisplayMode::STATE || mode == DisplayMode::BOTH) {
            if (state_ok && state_shm.read(state)) {
                display_state(state);
            } else {
                std::cout << "\n[ERROR] Failed to read state" << std::endl;
            }
        }
        
        if (mode == DisplayMode::COMMAND || mode == DisplayMode::BOTH) {
            if (cmd_ok && cmd_shm.read(cmd)) {
                display_command(cmd);
            } else {
                std::cout << "\n[ERROR] Failed to read command" << std::endl;
            }
        }
        
        if (mode == DisplayMode::BOTH) {
            std::cout << "\n========================================" << std::endl;
        }
        
        std::cout << std::flush;
        
        // Update rate
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    std::cout << "\nExiting..." << std::endl;
    return 0;
}
