#pragma once

#include <memory>
#include "ipc/shared_memory.h"
#include "ipc/command_channel.h"
#include "types/drone_state.h"
#include "types/drone_command.h"

namespace terrain_follower {
namespace control {

class OffboardMode {
public:
    OffboardMode();
    ~OffboardMode();
    
    // Initialize
    bool init();
    
    // Enter offboard mode at current position
    bool enter_offboard_mode();
    
    // Hold current position
    bool hold_position();
    
    // Get current position for hovering
    bool get_current_position(float& north, float& east, float& down);
    
private:
    std::unique_ptr<ipc::SharedMemory<ipc::DroneState>> state_shm_;
    std::unique_ptr<ipc::CommandChannel> cmd_channel_;
};

} // namespace control
} // namespace terrain_follower
