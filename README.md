# PX4 Terrain Follower

Autonomous terrain following system using PX4 SITL via MAVLink with behavior tree control and shared memory IPC.

## Prerequisites

**Note**: Due to GLIBC version requirements, MAVSDK needs to be built from source on Ubuntu 22.04.

### Install MAVSDK from source:
```bash
# Install dependencies
sudo apt update
sudo apt install build-essential cmake git libcurl4-openssl-dev libtinyxml2-dev libjsoncpp-dev

# Clone and build MAVSDK
cd /tmp
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK
cmake -Bbuild/default -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build/default -j$(nproc)
sudo cmake --build build/default --target install
sudo ldconfig
```

### Install BehaviorTree.CPP:
```bash
# Install dependencies
sudo apt install libzmq3-dev libboost-dev

# Clone and build
cd /tmp
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build && cd build

# Install to local third_party directory (adjust path to your project location)
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$(pwd)/../../third_party/behaviortree

make -j$(nproc)
make install
```

## Build

```bash
cd <project_directory>
mkdir build && cd build
cmake ..
make
```

## Usage

### 1. Start PX4 SITL
```bash
# Terminal 1
cd ~/PX4-Autopilot  # or your PX4 directory
make px4_sitl gazebo
```

### 2. Run MAVLink Interface
```bash
# Terminal 2
cd <project_directory>/build
./px4_interface/px4_mavlink_interface
```

This will:
- Connect to PX4 SITL on UDP port 14540
- Subscribe to all telemetry data
- Write drone state to shared memory `/px4_drone_state`
- Read commands from shared memory `/px4_drone_command`
- Execute flight commands (ARM, TAKEOFF, LAND, OFFBOARD, HOLD)

### 3. Run Behavior Tree Controller
```bash
# Terminal 3
cd <project_directory>/build
./control/bt_main
```

This runs the autonomous flight controller with:
- Reactive behavior tree for mode switching
- Support for TAKEOFF, OFFBOARD, HOLD, and LAND modes
- Event-driven mode changes via blackboard
- Groot2 visualization on port 1667

### 4. (Optional) Monitor State
```bash
# Terminal 4
cd <project_directory>/build
./debug/state_reader
```

This reads and displays the current drone state from shared memory.

## Architecture

**Single MAVLink Connection:**
- `px4_mavlink_interface` maintains sole connection to PX4
- All MAVSDK plugins (Telemetry, Action, Offboard)
- Command execution and telemetry streaming

**Shared Memory IPC:**
- State: `/px4_drone_state` (208 bytes, updated at 20Hz)
- Command: `/px4_drone_command` (command queue with semaphore sync)
- Thread-safe read/write operations
- POSIX shared memory + semaphores

**Behavior Tree Control:**
- ReactiveFallback state machine with blackboard
- Mode switching: TAKEOFF → OFFBOARD → HOLD → LAND
- Event-driven transitions
- GVisualization

Connect Groot2 to visualize the behavior tree in real-time:
```bash
# The bt_main process publishes on port 1667
# Groot2 should auto-discover the running tree
```

## Cleanup

If the shared memory doesn't clean up properly:
```bash
# Remove shared memory
sudo rm /dev/shm/px4_drone_state
sudo rm /dev/shm/px4_drone_command
sudo rm /dev/shm/sem.px4_drone_state_sem
sudo rm /dev/shm/sem.px4_drone_command_sem
```

## Project Structure

```
terrain_follower/
├── common/
│   ├── types/          # DroneState, DroneCommand structures
│   └── ipc/            # Shared memory wrapper templates
├── px4_interface/      # MAVLink connection and command execution
├── control/            # Behavior tree controller and flight modes
│   ├── include/
│   ├── src/
│   └── behavior_trees/ # XML definitions
├── debug/              # IPC monitoring utilities
└── third_party/        # Local dependencies (BehaviorTree.CPP)
```

