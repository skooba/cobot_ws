# COBOT ROS2 Workspace

This repository contains ROS2 packages for controlling a collaborative robot (cobot) system.

## Project Structure

```
src/
├── cobot_bringup/           # Launch files and system bringup
├── cobot_control_pkg/       # Main robot control logic
└── keyboard_controller_pkg/ # Keyboard-based robot control
```

## Prerequisites

- ROS2 Jazzy
- Python 3.10+ (ROS2 Jazzy requirement)
- Universal Robots UR5

**Note**: This project has been tested and verified to work on Linux systems only. Windows and macOS compatibility is not guaranteed.

## Installation
### 1. Clone the Repository
```bash
cd ~
git clone https://github.com/skooba/cobot_ws cobot_ws
```

### 2. Install System Dependencies
#### Required ROS2 Packages
```bash
# State machine libraries
sudo apt install ros-jazzy-smach ros-jazzy-smach-ros

# Universal Robots driver and dependencies
sudo apt install ros-jazzy-ur-robot-driver

# ROS2 Control (if not already installed)
sudo apt install ros-jazzy-ros2-control

# Additional message interfaces (usually included with ROS2)
sudo apt install ros-jazzy-example-interfaces

# Testing dependencies
sudo apt install python3-pytest
pip3 install pytest-mock
```

### 3. Build the Workspace
```bash
colcon build
```

### 4. Source the Workspace
```bash
source install/setup.bash
```

## Usage

### Starting the Cobot System

Launch the complete cobot system:
```bash
ros2 launch cobot_bringup cobot_control_viz.launch.py
```

**Launch Sequence and Timing:**
After running the launch command, the system follows this sequence:

1. **UR Driver Initialization (0-8 seconds):** The UR driver launches in simulation mode with RViz visualization. The UR5 controllers take approximately 8 seconds to fully initialize.

2. **Controller Activation (8 seconds):** The `forward_velocity_controller` is automatically activated after the 8-second delay built into the launch file.

3. **Cobot Nodes Startup (10 seconds):** An additional 2-second delay ensures the controller is fully ready before launching the custom cobot control nodes.

**Expected Behavior:** After running the launch command, you should expect a **12-15 second delay** before the custom written cobot control nodes begin running and the RViz state visualization starts moving. This delay is normal and ensures all components are properly initialized.

### Emergency Stop Control

To control the emergency stop functionality, open a new terminal from the complete cobot system:
```bash
ros2 run keyboard_controller_pkg keyboard_input_monitor
```

**Emergency Stop Controls:**
- Press `e` to activate emergency stop
- Press `r` to reset emergency stop
- Press `Ctrl+C` to exit

## Nodes and Topics

### cobot_control_pkg

#### ur_robot_controller
Main robot controller that handles velocity commands and safety states.

##### Published Topics:
- `/forward_velocity_controller/commands` (std_msgs/Float64MultiArray) - Joint velocity commands for all 6 UR robot joints

##### Subscribed Topics: 
- `/robot_speed_state` (example_interfaces/String) - Speed state commands (stopped, slow_speed, full_speed, estop)

##### Features:
- Velocity-based joint control for all 6 UR robot joints
- Speed state management (stopped, slow, full speed)
- Emergency stop handling
- Smooth velocity ramping with configurable ramp rates

#### speed_control
State machine that manages robot speed based on proximity and emergency stop status.

##### Published Topics:
- `/robot_speed_state` (example_interfaces/String) - Current speed state (stopped, slow_speed, full_speed, estop)

##### Subscribed Topics:
- `/proximity_distance` (example_interfaces/Float32) - Distance readings from proximity sensor
- `/emergency_stop_status` (example_interfaces/Bool) - Emergency stop status

##### Features:
- Finite state machine for speed control
- Monitors proximity sensor and emergency stop concurrently
- Automatically adjusts robot speed based on safety conditions

#### emergency_stop
Handles emergency stop logic and status publishing.

##### Published Topics:
- `/emergency_stop_status` (example_interfaces/Bool) - Emergency stop status (true = active, false = inactive)

##### Subscribed Topics:
- `/keyboard_input_monitor` (example_interfaces/Char) - Keyboard input for emergency stop control

##### Features:
- Processes keyboard commands for emergency stop activation/reset
- Publishes emergency stop status to other nodes

#### proximity_sensor
Simulates proximity sensor readings for safety monitoring.

##### Published Topics:
- `/proximity_distance` (example_interfaces/Float32) - Distance readings in millimeters

##### Configurable Parameters:
- `min_distance_mm` (default: 100.0) - Minimum distance reading
- `max_distance_mm` (default: 1000.0) - Maximum distance reading
- `publish_rate_hz` (default: 0.25) - Publishing frequency

**Note:** The launch file starts nodes with default parameters. Use `ros2 param set /proximity_sensor {param_name} {value}` to change parameters while the node is running.

### keyboard_controller_pkg

#### keyboard_input_monitor
Monitors keyboard input for robot control and emergency stop.

##### Published Topics:
- `/keyboard_input_monitor` (example_interfaces/Char) - ASCII values of pressed keys

##### Configurable Parameters:
- `key_timeout` (default: 0.1) - Timeout for key reading

**Note:** The launch file starts nodes with default parameters. Use `ros2 param set /keyboard_input_monitor {param_name} {value}` to change parameters while the node is running.

##### Features:
- Non-blocking keyboard input monitoring
- Cross-platform support (Windows and Linux)
- Publishes ASCII values of pressed keys

##### Reference:
- This node was adapted from the [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard) package

## Safety Features

### Emergency Stop System
- **Activation**: Press `e` in the keyboard input monitor
- **Reset**: Press `r` in the keyboard input monitor
- **Effect**: Immediately stops all robot motion and prevents further movement until reset

### Proximity-Based Speed Control
- **Close Range** (< 400mm): Robot stops
- **Medium Range** (400-800mm): Robot operates at slow speed
- **Safe Range** (> 800mm): Robot operates at full speed

### Velocity Ramping as Hysteresis
**Configurable Parameters:**
- `ramp_rate` (default: 0.5) - Maximum change in velocity of a joint per second during a ramp
- `frequency` (default: 10)  - Number of commands sent to the cobot per second during a ramp

**Note:** The launch file starts nodes with default parameters. Use `ros2 param set /ur_robot_controller {param_name} {value}` to change parameters while the node is running.

- Smooth acceleration/deceleration to prevent jerky motion
- Configurable ramp rates for different speed transitions
- Thread-safe velocity command handling

## Joint Configuration

The system is configured for Universal Robots with 6 joints:
1. `shoulder_pan_joint` (index 0)
2. `shoulder_lift_joint` (index 1)
3. `elbow_joint` (index 2)
4. `wrist_1_joint` (index 3)
5. `wrist_2_joint` (index 4)
6. `wrist_3_joint` (index 5)

## Testing

This project includes comprehensive unit and integration tests for all ROS2 nodes to ensure reliable operation and maintainability.

### Test Coverage

#### cobot_control_pkg
**Full coverage testing** has been completed for all nodes in this package:
- `proximity_sensor`: Parameter validation, publishing rate verification, publishing distance data verification, callback testing
- `emergency_stop`: Emergency activation/reset, keyboard input handling, state management, publishing estop data verification
- `speed_control`: State machine logic, concurrent monitoring, emergency stop priority, thread safety
- `speed_state_machine`: Speed decision logic, boundary conditions, individual state execution, constants validation
- `ur_robot_controller`: Velocity control, parameter validation, emergency stop procedure, thread management, ramping functionality

#### keyboard_controller_pkg
**Basic testing** with mocked resources has been completed:
- `keyboard_input_monitor`: Parameter validation, node initialization (I/O-dependent functionality tested via integration testing)

### Running Tests

#### Run All Tests
```bash
# Run all tests across all packages
colcon test

#### Run Tests for Specific Package
```bash
# Test specific package
colcon test --packages-select <package_name>

```
#### View Test Results
```bash
# Review detailed test results after running tests
colcon test-result --verbose

# View results for a specific package
colcon test-result --verbose --packages-select <package_name>
```

#### Run Individual Tests
```bash
# Run a specific test file
python3 -m pytest src/<package_name>/test/<test_file_name>.py -v

# Run a specific test class
python3 -m pytest src/<package_name>/test/<test_file_name>.py::<TestClassName> -v

# Run a specific test method
python3 -m pytest src/<package_name>/test/<test_file_name>.py::<TestClassName>::<test_method_name> -v

# Run tests with additional output (shows print statements)
python3 -m pytest src/<package_name>/test/<test_file_name>.py -v -s
```

**Note:** Test dependencies are included in the system dependencies installation above.

### Continuous Integration

All tests are designed to run reliably in CI/CD environments without requiring:
- Physical robot hardware
- Real keyboard/terminal input
- Platform-specific resources (tests work on both Linux and Windows)

## Development Notes

This project was primarily developed independently, with **Cursor AI** providing assistance for:
- Code style and formatting standardization (flake8/PEP257 compliance)
- Comprehensive unit test development and validation
- Documentation review and technical writing support

All core functionality, system architecture, ROS2 integration, and robotics logic were designed and implemented independently.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
