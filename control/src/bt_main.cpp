#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <cstring>

#include "flight_mode_controller.h"
#include "ipc/shared_memory.h"
#include "types/drone_command.h"

using namespace BT;
using namespace terrain_follower;

// Global flight controller
std::shared_ptr<control::FlightModeController> g_fc;
std::atomic<bool> g_takeoff_requested{false};
std::atomic<bool> g_land_requested{false};
std::atomic<bool> g_offboard_requested{false};
std::atomic<bool> g_hold_requested{false};
std::atomic<bool> g_should_exit{false};

void signal_handler(int signum) {
    std::cout << "\nReceived signal " << signum << ", exiting..." << std::endl;
    g_should_exit = true;
}

// ============ Custom BT Nodes ============

// ModeChangeRequested: Condition that checks for mode change and outputs requested_mode
class ModeChangeRequested : public BT::ConditionNode {
public:
    ModeChangeRequested(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    BT::NodeStatus tick() override {
        std::string requested_mode = "NONE";
        
        // Check flags in priority order: LAND > OFFBOARD > HOLD > TAKEOFF
        if (g_land_requested) {
            requested_mode = "LAND";
            g_land_requested = false;  // Consume the flag
        } else if (g_offboard_requested) {
            requested_mode = "OFFBOARD";
            g_offboard_requested = false;  // Consume the flag
        } else if (g_hold_requested) {
            requested_mode = "HOLD";
            g_hold_requested = false;  // Consume the flag
        } else if (g_takeoff_requested) {
            requested_mode = "TAKEOFF";
            g_takeoff_requested = false;  // Consume the flag
        }
        
        if (requested_mode != "NONE") {
            std::cout << "[BT] Mode change detected: " << requested_mode << std::endl;
            setOutput("requested_mode", requested_mode);
            return NodeStatus::SUCCESS;
        }
        
        return NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return { BT::OutputPort<std::string>("requested_mode") };
    }
};

// CheckBlackboard: Checks if blackboard key matches expected value
class CheckBlackboard : public BT::DecoratorNode {
public:
    CheckBlackboard(const std::string& name, const BT::NodeConfig& config)
        : BT::DecoratorNode(name, config) {}

    BT::NodeStatus tick() override {
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
            // Key doesn't exist, set it to empty
            return NodeStatus::FAILURE;
        }
        
        if (current_value == value) {
            // Mode matches, execute child
            const BT::NodeStatus child_status = child_node_->executeTick();
            return child_status;
        }
        
        return NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::string>("key"),
            BT::InputPort<std::string>("value")
        };
    }
};

// ArmAction: Reusable arm action
class ArmAction : public BT::SyncActionNode {
public:
    ArmAction(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        std::cout << "[BT] Arming..." << std::endl;
        
        if (g_fc->is_armed()) {
            std::cout << "[BT] Already armed" << std::endl;
            return NodeStatus::SUCCESS;
        }
        
        if (g_fc->arm()) {
            // Wait for arm to complete
            std::this_thread::sleep_for(std::chrono::seconds(2));
            return NodeStatus::SUCCESS;
        }
        
        return NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() { return {}; }
};

// TakeoffModeBT: Composite mode node for takeoff sequence
class TakeoffModeBT : public BT::StatefulActionNode {
public:
    TakeoffModeBT(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), state_(State::TAKEOFF) {}

    BT::NodeStatus onStart() override {
        std::cout << "[BT] TakeoffMode: Starting..." << std::endl;
        
        // Check if already in air
        if (g_fc->is_in_air()) {
            float current_alt = g_fc->get_altitude();
            std::cout << "[BT] TakeoffMode: Already in air at " << current_alt << " m, skipping takeoff" << std::endl;
            state_ = State::HOVER;
        } else {
            state_ = State::TAKEOFF;
        }
        
        return NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        switch (state_) {
            case State::TAKEOFF: {
                std::cout << "[BT] TakeoffMode: Executing takeoff..." << std::endl;
                if (g_fc->takeoff(5.0f)) {
                    state_ = State::WAIT_ALTITUDE;
                } else {
                    return NodeStatus::FAILURE;
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
                // Continue hovering
                std::cout << "[BT] TakeoffMode: Hovering (altitude: " << g_fc->get_altitude() << " m)" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                return NodeStatus::RUNNING;
            }
        }
        
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[BT] TakeoffModeBT: Halted" << std::endl;
    }

    static BT::PortsList providedPorts() { return {}; }

private:
    enum class State {
        TAKEOFF,
        WAIT_ALTITUDE,
        HOVER
    };
    
    State state_;
};

// OffboardModeBT: Composite mode node for offboard control
class OffboardModeBT : public BT::StatefulActionNode {
public:
    OffboardModeBT(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), state_(State::ENTER) {}

    BT::NodeStatus onStart() override {
        std::cout << "[BT] OffboardMode: Starting..." << std::endl;
        state_ = State::ENTER;
        return NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        switch (state_) {
            case State::ENTER: {
                std::cout << "[BT] OffboardMode: Entering offboard position mode..." << std::endl;
                if (g_fc->start_offboard_position()) {
                    std::cout << "[BT] OffboardMode: Offboard mode activated!" << std::endl;
                    state_ = State::HOLD_POSITION;
                } else {
                    return NodeStatus::FAILURE;
                }
                break;
            }
            
            case State::HOLD_POSITION: {
                // Continue holding position
                g_fc->hover_position_offboard();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return NodeStatus::RUNNING;
            }
        }
        
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[BT] OffboardModeBT: Halted" << std::endl;
    }

    static BT::PortsList providedPorts() { return {}; }

private:
    enum class State {
        ENTER,
        HOLD_POSITION
    };
    
    State state_;
};

// LandModeBT: Composite mode node for landing sequence
class LandModeBT : public BT::StatefulActionNode {
public:
    LandModeBT(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), state_(State::LANDING) {}

    BT::NodeStatus onStart() override {
        std::cout << "[BT] LandMode: Starting landing sequence..." << std::endl;
        state_ = State::LANDING;
        return NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        switch (state_) {
            case State::LANDING: {
                if (!landing_initiated_) {
                    std::cout << "[BT] LandMode: Executing land command..." << std::endl;
                    if (g_fc->land()) {
                        landing_initiated_ = true;
                        state_ = State::WAIT_LANDED;
                    } else {
                        return NodeStatus::FAILURE;
                    }
                }
                break;
            }
            
            case State::WAIT_LANDED: {
                if (!g_fc->is_in_air()) {
                    std::cout << "[BT] LandMode: Landed successfully!" << std::endl;
                    // Clear mode from blackboard to return to idle
                    config().blackboard->set("active_mode", std::string("IDLE"));
                    landing_initiated_ = false;
                    return NodeStatus::SUCCESS;
                } else {
                    std::cout << "[BT] LandMode: Waiting to land (altitude: " << g_fc->get_altitude() << " m)" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                break;
            }
        }
        
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[BT] LandModeBT: Halted" << std::endl;
        landing_initiated_ = false;
    }

    static BT::PortsList providedPorts() { return {}; }

private:
    enum class State {
        LANDING,
        WAIT_LANDED
    };
    
    State state_;
    bool landing_initiated_ = false;
};

// HoldModeBT: PX4 native HOLD mode
class HoldModeBT : public BT::StatefulActionNode {
public:
    HoldModeBT(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), state_(State::ENTER_HOLD) {}

    BT::NodeStatus onStart() override {
        std::cout << "[BT] HoldMode: Activating PX4 HOLD mode..." << std::endl;
        state_ = State::ENTER_HOLD;
        return NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        switch (state_) {
            case State::ENTER_HOLD: {
                if (g_fc->hold()) {
                    std::cout << "[BT] HoldMode: HOLD mode activated" << std::endl;
                    state_ = State::HOLDING;
                } else {
                    std::cerr << "[BT] HoldMode: Failed to activate HOLD" << std::endl;
                    return NodeStatus::FAILURE;
                }
                break;
            }
            
            case State::HOLDING: {
                // Continue in HOLD mode
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                return NodeStatus::RUNNING;
            }
        }
        
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[BT] HoldModeBT: Halted" << std::endl;
    }

    static BT::PortsList providedPorts() { return {}; }

private:
    enum class State {
        ENTER_HOLD,
        HOLDING
    };
    
    State state_;
};

// IdleMode: Default idle state
class IdleMode : public BT::StatefulActionNode {
public:
    IdleMode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    BT::NodeStatus onStart() override {
        return NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        // Just return RUNNING to keep the tree alive
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return NodeStatus::RUNNING;
    }

    void onHalted() override {}

    static BT::PortsList providedPorts() { return {}; }
};

// EmergencyAbort: Emergency condition check
class EmergencyAbort : public BT::ConditionNode {
public:
    EmergencyAbort(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    BT::NodeStatus tick() override {
        // Check for emergency conditions
        if (g_should_exit) {
            std::cout << "[BT] EMERGENCY ABORT!" << std::endl;
            g_fc->land();
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() { return {}; }
};

// ============ Main ============

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "Behavior Tree - Drone Controller" << std::endl;
    std::cout << "=================================" << std::endl;
    
    // Create flight mode controller
    g_fc = std::make_shared<terrain_follower::control::FlightModeController>();
    
    if (!g_fc->init()) {
        std::cerr << "Failed to initialize flight mode controller!" << std::endl;
        return 1;
    }
    
    // Create BT factory and register custom nodes
    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<ModeChangeRequested>("ModeChangeRequested");
    factory.registerNodeType<CheckBlackboard>("CheckBlackboard");
    factory.registerNodeType<ArmAction>("Arm");
    factory.registerNodeType<TakeoffModeBT>("TakeoffMode");
    factory.registerNodeType<OffboardModeBT>("OffboardMode");
    factory.registerNodeType<LandModeBT>("LandMode");
    factory.registerNodeType<HoldModeBT>("HoldMode");
    factory.registerNodeType<IdleMode>("IdleMode");
    factory.registerNodeType<EmergencyAbort>("EmergencyAbort");
    
    // Load BT from XML file
    std::string xml_file = "control/behavior_trees/autonomy.xml";
    std::cout << "\nLoading behavior tree from: " << xml_file << std::endl;
    
    auto tree = factory.createTreeFromFile(xml_file);
    
    // Create Groot2 publisher for tree visualization
    // Groot2 will connect to tcp://localhost:1667 by default
    Groot2Publisher publisher(tree);
    
    std::cout << "\nBehavior Tree loaded!" << std::endl;
    std::cout << "\n[Groot2 Visualization]" << std::endl;
    std::cout << "  Publisher running on default port (1667)" << std::endl;
    std::cout << "  Open Groot2 app - it will automatically discover this tree" << std::endl;
    std::cout << "  Download Groot2: https://www.behaviortree.dev/groot" << std::endl;
    
    std::cout << "\nCommands:" << std::endl;
    std::cout << "  t - Takeoff mode" << std::endl;
    std::cout << "  o - Offboard mode" << std::endl;
    std::cout << "  h - Hold mode" << std::endl;
    std::cout << "  l - Land mode" << std::endl;
    std::cout << "  q - Quit" << std::endl;
    std::cout << "\nEnter command: ";
    
    // Command input thread
    std::thread input_thread([&]() {
        std::string cmd;
        while (!g_should_exit) {
            std::cin >> cmd;
            if (cmd == "t" || cmd == "T") {
                g_takeoff_requested = true;
                std::cout << "Takeoff mode requested!\n> ";
            } else if (cmd == "o" || cmd == "O") {
                g_offboard_requested = true;
                std::cout << "Offboard mode requested!\n> ";
            } else if (cmd == "h" || cmd == "H") {
                g_hold_requested = true;
                std::cout << "Hold mode requested!\n> ";
            } else if (cmd == "l" || cmd == "L") {
                g_land_requested = true;
                std::cout << "Land mode requested!\n> ";
            } else if (cmd == "q" || cmd == "Q") {
                g_should_exit = true;
            }
        }
    });
    
    // BT execution loop
    while (!g_should_exit) {
        NodeStatus status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    input_thread.join();
    
    std::cout << "\nStopping Groot2 publisher..." << std::endl;
    // Groot2Publisher destructor will handle cleanup
    
    std::cout << "Shutting down..." << std::endl;
    return 0;
}
