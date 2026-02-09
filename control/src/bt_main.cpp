#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

#include "bt_mode_nodes.h"
#include "bt_mission_nodes.h"
#include "flight_mode_controller.h"

using namespace BT;
using namespace terrain_follower;

void signal_handler(int signum) {
    std::cout << "\nReceived signal " << signum << ", exiting..." << std::endl;
    bt::g_should_exit = true;
}

// ============ Main ============

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "Behavior Tree - Drone Controller" << std::endl;
    std::cout << "=================================" << std::endl;
    
    // Create flight mode controller
    bt::g_fc = std::make_shared<control::FlightModeController>();
    
    if (!bt::g_fc->init()) {
        std::cerr << "Failed to initialize flight mode controller!" << std::endl;
        return 1;
    }
    
    // Create BT factory and register custom nodes
    BT::BehaviorTreeFactory factory;
    
    // Register main BT nodes
    factory.registerNodeType<bt::ModeChangeRequested>("ModeChangeRequested");
    factory.registerNodeType<bt::CheckBlackboard>("CheckBlackboard");
    factory.registerNodeType<bt::ArmAction>("Arm");
    factory.registerNodeType<bt::EnterOffboard>("EnterOffboard");
    factory.registerNodeType<bt::StreamSetpointsFromSubtree>("StreamSetpointsFromSubtree");
    factory.registerNodeType<bt::TakeoffModeBT>("TakeoffMode");
    factory.registerNodeType<bt::LandModeBT>("LandMode");
    factory.registerNodeType<bt::HoldModeBT>("HoldMode");
    factory.registerNodeType<bt::IdleMode>("IdleMode");
    factory.registerNodeType<bt::EmergencyAbort>("EmergencyAbort");
    
    // Register mission nodes (used in offboard_missions.xml)
    factory.registerNodeType<bt::GenerateScanPath>("GenerateScanPath");
    factory.registerNodeType<bt::ExecuteWaypoints>("ExecuteWaypoints");
    factory.registerNodeType<bt::LoadDEM>("LoadDEM");
    factory.registerNodeType<bt::FollowTerrain>("FollowTerrain");
    factory.registerNodeType<bt::ManualSetpointControl>("ManualSetpointControl");
    
    // Load BT from XML file
    // Note: autonomy.xml includes offboard_missions.xml via <include> tag
    std::string xml_file = "control/behavior_trees/autonomy.xml";
    std::cout << "\nLoading behavior tree from: " << xml_file << std::endl;
    
    auto tree = factory.createTreeFromFile(xml_file);
    
    // Create Groot2 publisher for tree visualization
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
        while (!bt::g_should_exit) {
            std::cin >> cmd;
            if (cmd == "t" || cmd == "T") {
                bt::g_takeoff_requested = true;
                std::cout << "Takeoff mode requested!\n> ";
            } else if (cmd == "o" || cmd == "O") {
                bt::g_offboard_requested = true;
                std::cout << "Offboard mode requested!\n> ";
            } else if (cmd == "h" || cmd == "H") {
                bt::g_hold_requested = true;
                std::cout << "Hold mode requested!\n> ";
            } else if (cmd == "l" || cmd == "L") {
                bt::g_land_requested = true;
                std::cout << "Land mode requested!\n> ";
            } else if (cmd == "q" || cmd == "Q") {
                bt::g_should_exit = true;
            }
        }
    });
    
    // BT execution loop
    while (!bt::g_should_exit) {
        NodeStatus status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    input_thread.join();
    
    std::cout << "\nStopping Groot2 publisher..." << std::endl;
    std::cout << "Shutting down..." << std::endl;
    
    return 0;
}
