#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <memory>
#include <string>

namespace terrain_follower {
namespace control {
    class FlightModeController;
}

namespace bt {

// Extern reference to global flight controller (defined in bt_mode_nodes.cpp)
extern std::shared_ptr<control::FlightModeController> g_fc;

/**
 * @brief Manual setpoint control - holds current position
 * Outputs: setpoint_north, setpoint_east, setpoint_down, setpoint_yaw (float)
 */
class ManualSetpointControl : public BT::StatefulActionNode {
public:
    ManualSetpointControl(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
};

/**
 * @brief Generate scan path for area scanning mission
 * Inputs: bounds (string), altitude (float)
 */
class GenerateScanPath : public BT::SyncActionNode {
public:
    GenerateScanPath(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

/**
 * @brief Execute a list of waypoints
 * Outputs: setpoint_north, setpoint_east, setpoint_down, setpoint_yaw (float)
 */
class ExecuteWaypoints : public BT::StatefulActionNode {
public:
    ExecuteWaypoints(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
};

/**
 * @brief Load Digital Elevation Model for terrain following
 * Inputs: dem_file (string)
 */
class LoadDEM : public BT::SyncActionNode {
public:
    LoadDEM(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

/**
 * @brief Terrain following mission
 * Inputs: agl_altitude (float)
 * Outputs: setpoint_north, setpoint_east, setpoint_down, setpoint_yaw (float)
 */
class FollowTerrain : public BT::StatefulActionNode {
public:
    FollowTerrain(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
};

} // namespace bt
} // namespace terrain_follower
