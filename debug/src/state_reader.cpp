#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <cmath>

#include "types/drone_state.h"
#include "ipc/shared_memory.h"

using namespace terrain_follower;

std::atomic<bool> should_exit{false};

void signal_handler(int signum) {
    should_exit = true;
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "PX4 State Reader - Reading from shared memory..." << std::endl;
    
    // Open shared memory
    ipc::SharedMemory<ipc::DroneState> shm("px4_drone_state");
    if (!shm.open()) {
        std::cerr << "Failed to open shared memory! " << std::endl;
        std::cerr << "Make sure px4_mavlink_interface is running first." << std::endl;
        return 1;
    }
    
    std::cout << "Connected to shared memory: /px4_drone_state" << std::endl;
    std::cout << "Press Ctrl+C to exit.\n" << std::endl;
    
    ipc::DroneState state;
    
    while (!should_exit) {
        if (!shm.read(state)) {
            std::cerr << "Failed to read from shared memory!" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        
        // Clear screen (optional - comment out if you prefer scrolling output)
        // std::cout << "\033[2J\033[1;1H";
        
        std::cout << "========================================" << std::endl;
        std::cout << "         PX4 DRONE STATE" << std::endl;
        std::cout << "========================================" << std::endl;
        
        // Connection status
        std::cout << "\n[CONNECTION]" << std::endl;
        std::cout << "  Connected:       " << (state.is_connected ? "YES" : "NO") << std::endl;
        std::cout << "  Telemetry Valid: " << (state.telemetry_valid ? "YES" : "NO") << std::endl;
        std::cout << "  Timestamp:       " << state.timestamp_us << " us" << std::endl;
        
        if (!state.telemetry_valid) {
            std::cout << "\nWaiting for telemetry data..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        // Position
        std::cout << "\n[POSITION - NED Frame]" << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  North:  " << std::setw(8) << state.position_n << " m" << std::endl;
        std::cout << "  East:   " << std::setw(8) << state.position_e << " m" << std::endl;
        std::cout << "  Down:   " << std::setw(8) << state.position_d << " m" << std::endl;
        
        // Velocity
        std::cout << "\n[VELOCITY - NED Frame]" << std::endl;
        std::cout << "  North:  " << std::setw(8) << state.velocity_n << " m/s" << std::endl;
        std::cout << "  East:   " << std::setw(8) << state.velocity_e << " m/s" << std::endl;
        std::cout << "  Down:   " << std::setw(8) << state.velocity_d << " m/s" << std::endl;
        
        // Attitude
        std::cout << "\n[ATTITUDE]" << std::endl;
        std::cout << "  Roll:   " << std::setw(8) << (state.roll * 180.0f / M_PI) << " deg" << std::endl;
        std::cout << "  Pitch:  " << std::setw(8) << (state.pitch * 180.0f / M_PI) << " deg" << std::endl;
        std::cout << "  Yaw:    " << std::setw(8) << (state.yaw * 180.0f / M_PI) << " deg" << std::endl;
        
        // Angular velocity
        std::cout << "\n[ANGULAR VELOCITY]" << std::endl;
        std::cout << "  Roll rate:  " << std::setw(8) << state.rollspeed << " rad/s" << std::endl;
        std::cout << "  Pitch rate: " << std::setw(8) << state.pitchspeed << " rad/s" << std::endl;
        std::cout << "  Yaw rate:   " << std::setw(8) << state.yawspeed << " rad/s" << std::endl;
        
        // Battery
        std::cout << "\n[BATTERY]" << std::endl;
        std::cout << std::setprecision(1);
        std::cout << "  Voltage:   " << state.battery_voltage << " V" << std::endl;
        std::cout << "  Remaining: " << state.battery_remaining << " %" << std::endl;
        
        // Status
        std::cout << "\n[STATUS]" << std::endl;
        std::cout << "  Armed:       " << (state.armed ? "YES" : "NO") << std::endl;
        std::cout << "  In Air:      " << (state.in_air ? "YES" : "NO") << std::endl;
        std::cout << "  Flight Mode: " << (int)state.flight_mode << std::endl;
        
        // GPS
        std::cout << "\n[GPS]" << std::endl;
        std::cout << std::setprecision(6);
        std::cout << "  Latitude:   " << state.latitude << " deg" << std::endl;
        std::cout << "  Longitude:  " << state.longitude << " deg" << std::endl;
        std::cout << std::setprecision(2);
        std::cout << "  Altitude:   " << state.altitude << " m (MSL)" << std::endl;
        std::cout << "  Fix Type:   " << (int)state.gps_fix << std::endl;
        std::cout << "  Satellites: " << (int)state.num_satellites << std::endl;
        
        // Update every 500ms
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    std::cout << "Exiting..." << std::endl;
    return 0;
}
