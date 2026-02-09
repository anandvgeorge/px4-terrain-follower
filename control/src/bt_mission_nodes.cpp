#include "bt_mission_nodes.h"
#include "bt_mode_nodes.h"
#include "flight_mode_controller.h"
#include <iostream>

namespace terrain_follower {
namespace bt {

// ============ ManualSetpointControl ============

ManualSetpointControl::ManualSetpointControl(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus ManualSetpointControl::onStart() {
    // Get current position and use as setpoint
    float n, e, d, yaw;
    if (!g_fc->get_position(n, e, d, yaw)) {
        std::cerr << "[ManualSetpointControl] Failed to get current position" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    
    // Set initial setpoint to current position
    config().blackboard->set("setpoint_north", n);
    config().blackboard->set("setpoint_east", e);
    config().blackboard->set("setpoint_down", d);
    config().blackboard->set("setpoint_yaw", yaw);
    
    std::cout << "[ManualSetpointControl] Holding position: N=" << n 
              << " E=" << e << " D=" << d << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ManualSetpointControl::onRunning() {
    // Keep outputting the same setpoint (hold position)
    // In the future, this could be updated to accept user commands
    return BT::NodeStatus::RUNNING;
}

void ManualSetpointControl::onHalted() {
    std::cout << "[ManualSetpointControl] Halted" << std::endl;
}

BT::PortsList ManualSetpointControl::providedPorts() { 
    return {}; 
}

// ============ GenerateScanPath ============

GenerateScanPath::GenerateScanPath(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus GenerateScanPath::tick() {
    std::cout << "[GenerateScanPath] Generating scan path (stub)" << std::endl;
    
    // Stub implementation - in the future, this will generate waypoints
    // based on bounds and contours
    
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList GenerateScanPath::providedPorts() {
    return {
        BT::InputPort<std::string>("bounds"),
        BT::InputPort<float>("altitude")
    };
}

// ============ ExecuteWaypoints ============

ExecuteWaypoints::ExecuteWaypoints(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus ExecuteWaypoints::onStart() {
    std::cout << "[ExecuteWaypoints] Starting waypoint execution (stub)" << std::endl;
    return BT::NodeStatus::SUCCESS;  // Stub - immediately complete
}

BT::NodeStatus ExecuteWaypoints::onRunning() {
    return BT::NodeStatus::SUCCESS;
}

void ExecuteWaypoints::onHalted() {
    std::cout << "[ExecuteWaypoints] Halted" << std::endl;
}

BT::PortsList ExecuteWaypoints::providedPorts() { 
    return {}; 
}

// ============ LoadDEM ============

LoadDEM::LoadDEM(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus LoadDEM::tick() {
    std::cout << "[LoadDEM] Loading DEM (stub)" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList LoadDEM::providedPorts() {
    return {
        BT::InputPort<std::string>("dem_file")
    };
}

// ============ FollowTerrain ============

FollowTerrain::FollowTerrain(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus FollowTerrain::onStart() {
    std::cout << "[FollowTerrain] Starting terrain following (stub)" << std::endl;
    return BT::NodeStatus::SUCCESS;  // Stub - immediately complete
}

BT::NodeStatus FollowTerrain::onRunning() {
    return BT::NodeStatus::SUCCESS;
}

void FollowTerrain::onHalted() {
    std::cout << "[FollowTerrain] Halted" << std::endl;
}

BT::PortsList FollowTerrain::providedPorts() {
    return {
        BT::InputPort<float>("agl_altitude")
    };
}

} // namespace bt
} // namespace terrain_follower
