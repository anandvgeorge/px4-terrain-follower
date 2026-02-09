#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <memory>
#include <atomic>
#include <string>
#include <chrono>

namespace terrain_follower {
namespace control {
    class FlightModeController;
}

namespace bt {

// Global flight controller (extern declaration)
extern std::shared_ptr<control::FlightModeController> g_fc;
extern std::atomic<bool> g_takeoff_requested;
extern std::atomic<bool> g_land_requested;
extern std::atomic<bool> g_offboard_requested;
extern std::atomic<bool> g_hold_requested;
extern std::atomic<bool> g_should_exit;

/**
 * @brief Checks for mode change requests from user input
 * Outputs: requested_mode (string)
 */
class ModeChangeRequested : public BT::ConditionNode {
public:
    ModeChangeRequested(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

/**
 * @brief Decorator that executes child only if blackboard key matches value
 * Inputs: key (string), value (string)
 */
class CheckBlackboard : public BT::DecoratorNode {
public:
    CheckBlackboard(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

/**
 * @brief Arms the drone
 */
class ArmAction : public BT::SyncActionNode {
public:
    ArmAction(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

/**
 * @brief Enters offboard position mode
 */
class EnterOffboard : public BT::SyncActionNode {
public:
    EnterOffboard(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

/**
 * @brief Decorator that streams setpoints while ticking child SubTree
 * Wraps a SubTree node and streams position setpoints at controlled rate
 * Uses separate thread for guaranteed rate-controlled streaming
 */
class StreamSetpointsFromSubtree : public BT::DecoratorNode {
public:
    StreamSetpointsFromSubtree(const std::string& name, const BT::NodeConfig& config);
    ~StreamSetpointsFromSubtree();
    BT::NodeStatus tick() override;
    void halt() override;
    static BT::PortsList providedPorts();

private:
    int setpoint_rate_hz_;
    std::atomic<bool> streaming_active_;
    std::unique_ptr<std::thread> streaming_thread_;
    
    void start_streaming_thread();
    void stop_streaming_thread();
};

/**
 * @brief Takeoff mode - arms, takes off, and hovers
 */
class TakeoffModeBT : public BT::StatefulActionNode {
public:
    TakeoffModeBT(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    enum class State {
        TAKEOFF,
        WAIT_ALTITUDE,
        HOVER
    };
    State state_;
};

// Note: OffboardModeBT removed - replaced with EnterOffboard + StreamSetpointsFromSubtree + SubTree pattern

/**
 * @brief Land mode - initiates landing and waits for touchdown
 */
class LandModeBT : public BT::StatefulActionNode {
public:
    LandModeBT(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    enum class State {
        LANDING,
        WAIT_GROUNDED,
        DISARM
    };
    State state_;
};

/**
 * @brief Hold mode - activates PX4 hold mode
 */
class HoldModeBT : public BT::StatefulActionNode {
public:
    HoldModeBT(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    enum class State {
        ACTIVATING,
        HOLDING
    };
    State state_;
};

/**
 * @brief Idle mode - does nothing, waiting for commands
 */
class IdleMode : public BT::StatefulActionNode {
public:
    IdleMode(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
};

/**
 * @brief Emergency abort - lands immediately on exit signal
 */
class EmergencyAbort : public BT::ConditionNode {
public:
    EmergencyAbort(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

} // namespace bt
} // namespace terrain_follower
