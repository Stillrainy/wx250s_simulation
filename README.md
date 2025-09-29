# WX250s Robotic Arm Simulation

This project provides simulation and control capabilities for the Interbotix WX250s robotic arm using MATLAB's Robotics System Toolbox. The project includes URDF models, 3D visualizations, and forward kinematics calculations.

## Features

- **Robot Model**: Complete URDF model of the WX250s robotic arm with 6 DOF + gripper
- **3D Visualization**: Interactive robot visualization using MATLAB's `show()` function
- **Forward Kinematics**: Joint angle to end-effector pose calculations using `rigidBodyTree`
- **Joint Limit Checking**: Automatic validation of joint positions against limits
- **MATLAB Class Interface**: Object-oriented design for easy robot control

## Prerequisites

- **Operating System**: macOS, Linux, or Windows
- **MATLAB**: R2019b or later (recommended R2020a+)
- **Robotics System Toolbox**: Required for URDF loading and kinematics calculations

## Installation

### 1. Clone/Download the Project

Ensure you have the project files in your working directory with the following structure:
```
wx250s_simulation/
├── wx250s.m
├── test_robot.m
└── assets/
    ├── wx250s.urdf
    └── meshes/
        └── *.stl files
```

### 2. MATLAB Setup

1. **Install Required Toolbox**:
   - Open MATLAB
   - Go to Home tab → Add-Ons → Get Add-Ons
   - Search for "Robotics System Toolbox" and install it

2. **Verify Installation**:
   ```matlab
   % Check if Robotics System Toolbox is installed
   ver robotics
   ```

3. **Set Working Directory**:
   Navigate to the project directory in MATLAB or use:
   ```matlab
   cd('path/to/wx250s_simulation')
   ```

## Usage

### Basic Example

```matlab
% Initialize the robot
robot = wx250s();

% Move to home position
robot.goToHomePose();

% Set joint positions (in radians)
joint_positions = deg2rad([0, -35, 60, 0, 65, 0]);
robot.setJointPositions(joint_positions);

% Get current joint positions
current_pos = robot.getJointPositions();

% Get end-effector pose
ee_pose = robot.getEePose();

% Visualize the robot
robot.showRobot();
```

### Running the Test Script

Execute the comprehensive test script:
```matlab
run('test_robot.m')
```

This script demonstrates:
- Robot initialization
- Joint limit checking
- Forward kinematics calculations
- 3D visualization

### Available Methods

#### `wx250s(urdf_path)`
Constructor that loads the URDF model.
- `urdf_path` (optional): Path to URDF file (default: 'assets/wx250s.urdf')

#### `setJointPositions(joint_positions)`
Set joint positions with automatic limit checking.
- `joint_positions`: 6-element vector of joint angles in radians
- Returns: Boolean indicating success

#### `getJointPositions()`
Get current joint positions.
- Returns: 6-element column vector of joint angles

#### `getEePose()`
Get end-effector pose using forward kinematics.
- Returns: 4x4 homogeneous transformation matrix

#### `goToHomePose()`
Move robot to home position (all joints at zero).

#### `showRobot()`
Visualize the robot in its current configuration.

## Resources

- [Interbotix WX250s Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx250s.html)
- [MATLAB Robotics System Toolbox Documentation](https://www.mathworks.com/products/robotics.html)

## Acknowledgements

The URDF was adopted from the [interbotix_ros_manipulators](https://github.com/Interbotix/interbotix_ros_manipulators/tree/rolling/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf) repository.

