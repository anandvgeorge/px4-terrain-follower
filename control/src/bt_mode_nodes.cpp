#include "bt_mode_nodes.h"
#include "bt_mission_nodes.h"
#include "flight_mode_controller.h"
#include <iostream>
#include <thread>
#include <chrono>

namespace terrain_follower {
namespace bt {

// Global definitions
std::shared_ptr<control::FlightModeController> g_fc;
std::atomic<bool> g_takeoff_requested{false};
std::atomic<bool> g_land_requested{false};
std::atomic<bool> g_offboard_requested{false};
std::atomic<bool> g_hold_requested{false};
std::atomic<bool> g_should_exit{false};

// ============ ModeChangeRequested ============

ModeChangeRequested::ModeChangeRequested(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::NodeStatus ModeChangeRequested::tick() {
    std::string requested_mode = "NONE";
    
    // Check flags in priority order: LAND > OFFBOARD > HOLD > TAKEOFF
    if (g_land_requested) {
        requested_mode = "LAND";
        g_land_requested = false;
    } else if (g_offboard_requested) {
        requested_mode = "OFFBOARD";
        g_offboard_requested = false;
    } else if (g_hold_requested) {
        requested_mode = "HOLD";
        g_hold_requested = false;
    } else if (g_takeoff_requested) {
        requested_mode = "TAKEOFF";
        g_takeoff_requested = false;
    }
    
    if (requested_mode != "NONE") {
        std::cout << "[BT] Mode change detected: " << requested_mode << std::endl;
        setOutput("requested_mode", requested_mode);
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}

BT::PortsList ModeChangeRequested::providedPorts() {
    return { BT::OutputPort<std::string>("requested_mode") };
}

// ============ CheckBlackboard ============

CheckBlackboard::CheckBlackboard(const std::string& name, const BT::NodeConfig& config)
    : BT::DecoratorNode(name, config) {}

BT::NodeStatus CheckBlackboard::tick() {
    std::string key;
    std::string value;
    
    if (!getInput("key", key)) {
        throw BT::RuntimeError("Missing parameter [key]");
    }
    if (!getInput("value", value)) {
        throw BT::RuntimeError("Missing parameter [value]");
    }
    
    // Check if blackboard value matches
    std::string current_value;
    if (!config().blackboard->get(key, current_value)) {
        return BT::NodeStatus::FAILURE;
    }
    
    if (current_value == value) {
        const BT::NodeStatus child_status = child_node_->executeTick();
        return child_status;
    }
    
    return BT::NodeStatus::FAILURE;
}

BT::PortsList CheckBlackboard::providedPorts() {
    return { 
        BT::InputPort<std::string>("key"),
        BT::InputPort<std::string>("value")
    };
}

// ============ ArmAction ============

ArmAction::ArmAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus ArmAction::tick() {
    std::cout << "[BT] Arming..." << std::endl;
    
    if (g_fc->is_armed()) {
        std::cout << "[BT] Already armed" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    
    if (g_fc->arm()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}

BT::PortsList ArmAction::providedPorts() { 
    return {}; 
}

// ============ EnterOffboard ============

EnterOffboard::EnterOffboard(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus EnterOffboard::tick() {
    std::cout << "[BT] EnterOffboard: Activating offboard position mode..." << std::endl;
    
    if (!g_fc->start_offboard_position()) {
        std::cerr << "[BT] EnterOffboard: Failed to enter offboard!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    
    std::cout << "[BT] EnterOffboard: Successfully entered offboard mode" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList EnterOffboard::providedPorts() { 
    return {}; 
}

// ============ StreamSetpointsFromSubtree ============

StreamSetpointsFromSubtree::StreamSetpointsFromSubtree(
    const std::string& name, const BT::NodeConfig& config)
    : BT::DecoratorNode(name, config), 
      setpoint_rate_hz_(20),
      streaming_active_(false) {}

StreamSetpointsFromSubtree::~StreamSetpointsFromSubtree() {
    stop_streaming_thread();
}

BT::NodeStatus StreamSetpointsFromSubtree::tick() {
    // Start async streaming thread on first tick
    if (!streaming_active_) {
        start_streaming_thread();
    }
    
    // Tick child (mission SubTree) - can be slow, doesn't block setpoints!
    setStatus(BT::NodeStatus::RUNNING);
    BT::NodeStatus child_status = child_node_->executeTick();
    
    return child_status;
}

void StreamSetpointsFromSubtree::halt() {
    std::cout << "[StreamSetpoints] Halted - stopping offboard mode" << std::endl;
    stop_streaming_thread();
    g_fc->stop_offboard();
    BT::DecoratorNode::halt();
}

BT::PortsList StreamSetpointsFromSubtree::providedPorts() {
    return {};
}

void StreamSetpointsFromSubtree::start_streaming_thread() {
    std::cout << "[StreamSetpoints] Starting async setpoint thread at " 
              << setpoint_rate_hz_ << " Hz" << std::endl;
    
    streaming_active_ = true;
    
    streaming_thread_ = std::make_unique<std::thread>([this]() {
        auto interval = std::chrono::milliseconds(1000 / setpoint_rate_hz_);
        
        while (streaming_active_) {
            auto start_time = std::chrono::steady_clock::now();
            
            // Read setpoints from blackboard (set by mission nodes)
            float n = 0.0f, e = 0.0f, d = 0.0f, yaw = 0.0f;
            
            if (config().blackboard->get("setpoint_north", n) &&
                config().blackboard->get("setpoint_east", e) &&
                config().blackboard->get("setpoint_down", d) &&
                config().blackboard->get("setpoint_yaw", yaw)) {
                
                // Send setpoint to PX4 (non-blocking)
                g_fc->set_position_offboard(n, e, d, yaw);
            }
            
            // Sleep for remaining time to maintain precise rate
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto sleep_time = interval - elapsed;
            
            if (sleep_time > std::chrono::milliseconds(0)) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
        
        std::cout << "[StreamSetpoints] Async thread stopped" << std::endl;
    });
}

void StreamSetpointsFromSubtree::stop_streaming_thread() {
    if (streaming_active_) {
        streaming_active_ = false;
        
        if (streaming_thread_ && streaming_thread_->joinable()) {
            streaming_thread_->join();
        }
        
        streaming_thread_.reset();
    }
}

// ============ TakeoffModeBT ============

TakeoffModeBT::TakeoffModeBT(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), state_(State::TAKEOFF) {}

BT::NodeStatus TakeoffModeBT::onStart() {
    std::cout << "[BT] TakeoffMode: Starting..." << std::endl;
    
    if (g_fc->is_in_air()) {
        float current_alt = g_fc->get_altitude();
        std::cout << "[BT] TakeoffMode: Already in air at " << current_alt 
                  << " m, skipping takeoff" << std::endl;
        state_ = State::HOVER;
    } else {
        state_ = State::TAKEOFF;
    }
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffModeBT::onRunning() {
    switch (state_) {
        case State::TAKEOFF: {
            std::cout << "[BT] TakeoffMode: Executing takeoff..." << std::endl;
            if (g_fc->takeoff(5.0f)) {
                state_ = State::WAIT_ALTITUDE;
            } else {
                return BT::NodeStatus::FAILURE;
            }
            break;
        }
        
        case State::WAIT_ALTITUDE: {
            float alt = g_fc->get_altitude();
            if (alt > 4.5f) {
                std::cout << "[BT] TakeoffMode: Altitude reached (" << alt << " m)" << std::endl;
                state_ = State::HOVER;
            } else {
                std::cout << "[BT] TakeoffMode: Current altitude: " << alt << " m" << std::endl;
            }
            break;
        }
        
        case State::HOVER: {
            std::cout << "[BT] TakeoffMode: Hovering (altitude: " 
                      << g_fc->get_altitude() << " m)" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return BT::NodeStatus::RUNNING;
        }
    }
    
    return BT::NodeStatus::RUNNING;
}

void TakeoffModeBT::onHalted() {
    std::cout << "[BT] TakeoffModeBT: Halted" << std::endl;
}

BT::PortsList TakeoffModeBT::providedPorts() { 
    return {}; 
}

// Note: OffboardModeBT removed - replaced with EnterOffboard + StreamSetpointsFromSubtree decorator pattern

// ============ LandModeBT ============

LandModeBT::LandModeBT(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), state_(State::LANDING) {}

BT::NodeStatus LandModeBT::onStart() {
    std::cout << "[BT] LandMode: Starting landing sequence..." << std::endl;
    state_ = State::LANDING;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LandModeBT::onRunning() {
    switch (state_) {
        case State::LANDING: {
            std::cout << "[BT] LandMode: Executing land command..." << std::endl;
            if (g_fc->land()) {
                state_ = State::WAIT_GROUNDED;
            } else {
                return BT::NodeStatus::FAILURE;
            }
            break;
        }
        
        case State::WAIT_GROUNDED: {
            if (!g_fc->is_in_air()) {
                std::cout << "[BT] LandMode: Landed successfully!" << std::endl;
                state_ = State::DISARM;
            } else {
                std::cout << "[BT] LandMode: Waiting to land (altitude: " 
                          << g_fc->get_altitude() << " m)" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            break;
        }
        
        case State::DISARM: {
            config().blackboard->set("active_mode", std::string("IDLE"));
            return BT::NodeStatus::SUCCESS;
        }
    }
    
    return BT::NodeStatus::RUNNING;
}

void LandModeBT::onHalted() {
    std::cout << "[BT] LandModeBT: Halted" << std::endl;
}

BT::PortsList LandModeBT::providedPorts() { 
    return {}; 
}

// ============ HoldModeBT ============

HoldModeBT::HoldModeBT(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), state_(State::ACTIVATING) {}

BT::NodeStatus HoldModeBT::onStart() {
    std::cout << "[BT] HoldMode: Activating PX4 HOLD mode..." << std::endl;
    state_ = State::ACTIVATING;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoldModeBT::onRunning() {
    switch (state_) {
        case State::ACTIVATING: {
            if (g_fc->hold()) {
                std::cout << "[BT] HoldMode: HOLD mode activated" << std::endl;
                state_ = State::HOLDING;
            } else {
                std::cerr << "[BT] HoldMode: Failed to activate HOLD" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            break;
        }
        
        case State::HOLDING: {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return BT::NodeStatus::RUNNING;
        }
    }
    
    return BT::NodeStatus::RUNNING;
}

void HoldModeBT::onHalted() {
    std::cout << "[BT] HoldModeBT: Halted" << std::endl;
}

BT::PortsList HoldModeBT::providedPorts() { 
    return {}; 
}

// ============ IdleMode ============

IdleMode::IdleMode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus IdleMode::onStart() {
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus IdleMode::onRunning() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return BT::NodeStatus::RUNNING;
}

void IdleMode::onHalted() {}

BT::PortsList IdleMode::providedPorts() { 
    return {}; 
}

// ============ EmergencyAbort ============

EmergencyAbort::EmergencyAbort(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::NodeStatus EmergencyAbort::tick() {
    if (g_should_exit) {
        std::cout << "[BT] EMERGENCY ABORT!" << std::endl;
        g_fc->land();
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList EmergencyAbort::providedPorts() { 
    return {}; 
}

} // namespace bt
} // namespace terrain_follower
