#pragma once

#include <memory>
#include <chrono>
#include "ipc/shared_memory.h"
#include "ipc/command_channel.h"
#include "types/drone_state.h"
#include "types/drone_command.h"

namespace terrain_follower {
namespace control {

class ControlNode {
public:
    ControlNode();
    ~ControlNode();
    
    // Initialize (open shared memory)
    bool init();
    
    // Actions (send commands via command channel)
    bool arm();
    bool takeoff(float altitude_m = 5.0f);
    bool land();
    bool disarm();
    bool hold();
    
    // State queries (from shared memory)
    bool is_armed();
    bool is_in_air();
    bool is_ready_to_arm();
    bool is_connected();
    
    // Get current altitude
    float get_altitude();
    
private:
    bool send_command(ipc::CommandType type, float param1 = 0.0f, 
                     int timeout_ms = 5000);
    
    std::unique_ptr<ipc::SharedMemory<ipc::DroneState>> state_shm_;
    std::unique_ptr<ipc::CommandChannel> cmd_channel_;
};

} // namespace control
} // namespace terrain_follower
