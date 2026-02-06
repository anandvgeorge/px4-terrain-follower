#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <future>
#include <cmath>
#include <cstring>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>

#include "types/drone_state.h"
#include "types/drone_command.h"
#include "ipc/shared_memory.h"

using namespace mavsdk;
using namespace terrain_follower;

std::atomic<bool> should_exit{false};

void signal_handler(int signum) {
    std::cout << "\nReceived signal " << signum << ", exiting..." << std::endl;
    should_exit = true;
}

int main(int argc, char* argv[]) {
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "PX4 MAVLink Interface Starting..." << std::endl;
    
    // Connection string (default PX4 SITL UDP port)
    std::string connection_url = "udp://:14540";
    if (argc > 1) {
        connection_url = argv[1];
    }
    
    // Create shared memory for state
    ipc::SharedMemory<ipc::DroneState> state_shm("px4_drone_state");
    if (!state_shm.create()) {
        std::cerr << "Failed to create state shared memory!" << std::endl;
        return 1;
    }
    std::cout << "State shared memory created: /px4_drone_state" << std::endl;
    
    // Create shared memory for commands
    ipc::SharedMemory<ipc::DroneCommand> cmd_shm("px4_drone_command");
    if (!cmd_shm.create()) {
        std::cerr << "Failed to create command shared memory!" << std::endl;
        return 1;
    }
    std::cout << "Command shared memory created: /px4_drone_command" << std::endl;
    
    // Initialize drone state
    ipc::DroneState state{};
    state.is_connected = false;
    state.telemetry_valid = false;
    state_shm.write(state);
    
    // Initialize command
    ipc::DroneCommand cmd{};
    cmd.type = ipc::CommandType::NONE;
    cmd.status = ipc::CommandStatus::IDLE;
    cmd_shm.write(cmd);
    
    // Create MAVSDK instance
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    
    // Add connection
    std::cout << "Connecting to: " << connection_url << std::endl;
    ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);
    
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }
    
    // Wait for system to connect
    std::cout << "Waiting for system to connect..." << std::endl;
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();
    
    Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
        auto system = mavsdk.systems().back();
        
        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot" << std::endl;
            mavsdk.unsubscribe_on_new_system(handle);
            prom.set_value(system);
    auto action = Action{system};
        }
    });
    
    // Wait for discovery with timeout
    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        std::cerr << "No autopilot found, timeout." << std::endl;
        return 1;
    }
    
    auto system = fut.get();
    std::cout << "Connected to PX4!" << std::endl;
    
    // Instantiate plugins
    auto telemetry = Telemetry{system};
    auto action = Action{system};
    auto offboard = Offboard{system};
    
    // Set telemetry rate (higher rate for better responsiveness)
    telemetry.set_rate_position(10.0);  // 10 Hz
    telemetry.set_rate_attitude_euler(20.0);  // 20 Hz
    telemetry.set_rate_velocity_ned(10.0);
    
    state.is_connected = true;
    
    // Subscribe to telemetry updates
    std::cout << "Subscribing to telemetry..." << std::endl;
    
    // Position
    telemetry.subscribe_position([&state](Telemetry::Position position) {
        state.latitude = position.latitude_deg;
        state.longitude = position.longitude_deg;
        state.altitude = position.absolute_altitude_m;
    });
    
    // Position in NED
    telemetry.subscribe_position_velocity_ned([&state](Telemetry::PositionVelocityNed pos_vel) {
        state.position_n = pos_vel.position.north_m;
        state.position_e = pos_vel.position.east_m;
        state.position_d = pos_vel.position.down_m;
        
        state.velocity_n = pos_vel.velocity.north_m_s;
        state.velocity_e = pos_vel.velocity.east_m_s;
        state.velocity_d = pos_vel.velocity.down_m_s;
    });
    
    // Attitude (Euler angles)
    telemetry.subscribe_attitude_euler([&state](Telemetry::EulerAngle attitude) {
        state.roll = attitude.roll_deg * M_PI / 180.0f;
        state.pitch = attitude.pitch_deg * M_PI / 180.0f;
        state.yaw = attitude.yaw_deg * M_PI / 180.0f;
    });
    
    // Angular velocity
    telemetry.subscribe_attitude_angular_velocity_body([&state](Telemetry::AngularVelocityBody ang_vel) {
        state.rollspeed = ang_vel.roll_rad_s;
        state.pitchspeed = ang_vel.pitch_rad_s;
        state.yawspeed = ang_vel.yaw_rad_s;
    });
    
    // Battery
    telemetry.subscribe_battery([&state](Telemetry::Battery battery) {
        state.battery_voltage = battery.voltage_v;
        state.battery_remaining = battery.remaining_percent * 100.0f;
    });
    
    // Armed status
    telemetry.subscribe_armed([&state](bool armed) {
        state.armed = armed;
    });
    
    // In-air status
    telemetry.subscribe_in_air([&state](bool in_air) {
        state.in_air = in_air;
    });
    
    // Flight mode
    telemetry.subscribe_flight_mode([&state](Telemetry::FlightMode flight_mode) {
        state.flight_mode = static_cast<uint8_t>(flight_mode);
    });
    
    // GPS info
    telemetry.subscribe_gps_info([&state](Telemetry::GpsInfo gps_info) {
        state.gps_fix = static_cast<uint8_t>(gps_info.fix_type);
        state.num_satellites = gps_info.num_satellites;
    });
    
    std::cout << "Telemetry streaming started. Press Ctrl+C to exit." << std::endl;
    std::cout << "Listening for commands on shared memory..." << std::endl;
    
    // Main loop - update shared memory and process commands
    auto last_update = std::chrono::steady_clock::now();
    
    while (!should_exit) {
        auto now = std::chrono::steady_clock::now();
        
        // Update timestamp
        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        state.timestamp_us = timestamp;
        state.telemetry_valid = true;
        
        // Write state to shared memory
        if (!state_shm.write(state)) {
            std::cerr << "Failed to write state to shared memory!" << std::endl;
        }
        
        // Check for commands
        if (cmd_shm.read(cmd)) {
            if (cmd.status == ipc::CommandStatus::PENDING) {
                std::cout << "\n[Command] Received: " << static_cast<int>(cmd.type) << std::endl;
                cmd.status = ipc::CommandStatus::EXECUTING;
                cmd_shm.write(cmd);
                
                bool success = false;
                std::string error_msg;
                
                switch (cmd.type) {
                    case ipc::CommandType::ARM: {
                        auto result = action.arm();
                        success = (result == Action::Result::Success);
                        if (!success) {
                            error_msg = "Arm failed";
                        }
                        break;
                    }
                    case ipc::CommandType::DISARM: {
                        auto result = action.disarm();
                        success = (result == Action::Result::Success);
                        if (!success) {
                            error_msg = "Disarm failed";
                        }
                        break;
                    }
                    case ipc::CommandType::TAKEOFF: {
                        auto result = action.set_takeoff_altitude(cmd.param1);
                        if (result == Action::Result::Success) {
                            result = action.takeoff();
                            success = (result == Action::Result::Success);
                        }
                        if (!success) {
                            error_msg = "Takeoff failed";
                        }
                        break;
                    }
                    case ipc::CommandType::LAND: {
                        auto result = action.land();
                        success = (result == Action::Result::Success);
                        if (!success) {
                            error_msg = "Land failed";
                        }
                        break;
                    }
                    case ipc::CommandType::OFFBOARD: {
                        // Start offboard mode with initial setpoint at current position
                        Offboard::PositionNedYaw initial_setpoint{
                            state.position_n,
                            state.position_e,
                            state.position_d,
                            state.yaw
                        };
                        auto result = offboard.set_position_ned(initial_setpoint);
                        if (result == Offboard::Result::Success) {
                            result = offboard.start();
                            success = (result == Offboard::Result::Success);
                        }
                        if (!success) {
                            error_msg = "Offboard mode failed";
                        }
                        break;
                    }
                    case ipc::CommandType::SET_POSITION: {
                        // Send position setpoint (offboard must already be active)
                        Offboard::PositionNedYaw setpoint{
                            cmd.param1,  // North
                            cmd.param2,  // East
                            cmd.param3,  // Down
                            cmd.param4   // Yaw
                        };
                        auto result = offboard.set_position_ned(setpoint);
                        success = (result == Offboard::Result::Success);
                        if (!success) {
                            error_msg = "Set position failed";
                        }
                        break;
                    }
                    case ipc::CommandType::HOLD: {
                        auto result = action.hold();
                        success = (result == Action::Result::Success);
                        if (!success) {
                            error_msg = "HOLD mode failed";
                        }
                        break;
                    }
                    default:
                        error_msg = "Unknown command";
                        break;
                }
                
                // Update command status
                cmd.status = success ? ipc::CommandStatus::SUCCESS : ipc::CommandStatus::FAILED;
                if (!success) {
                    std::strncpy(cmd.message, error_msg.c_str(), sizeof(cmd.message) - 1);
                    std::cerr << "[Command] " << error_msg << std::endl;
                } else {
                    std::cout << "[Command] Completed successfully" << std::endl;
                }
                cmd_shm.write(cmd);
            }
        }
        
        // Print status every 2 seconds
        // if (std::chrono::duration_cast<std::chrono::seconds>(now - last_update).count() >= 2) {
        //     std::cout << "\n--- Drone Status ---" << std::endl;
        //     std::cout << "Position (NED): [" 
        //               << state.position_n << ", " 
        //               << state.position_e << ", " 
        //               << state.position_d << "] m" << std::endl;
        //     std::cout << "Attitude (RPY): [" 
        //               << state.roll * 180.0f / M_PI << ", " 
        //               << state.pitch * 180.0f / M_PI << ", " 
        //               << state.yaw * 180.0f / M_PI << "] deg" << std::endl;
        //     std::cout << "Battery: " << state.battery_remaining << "% (" 
        //               << state.battery_voltage << "V)" << std::endl;
        //     std::cout << "Armed: " << (state.armed ? "YES" : "NO") 
        //               << " | In Air: " << (state.in_air ? "YES" : "NO") << std::endl;
        //     std::cout << "GPS: Fix=" << (int)state.gps_fix 
        //               << " Sats=" << (int)state.num_satellites << std::endl;
            
        //     last_update = now;
        // }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << "Shutting down..." << std::endl;
    state_shm.close();
    cmd_shm.close();
    
    return 0;
}
