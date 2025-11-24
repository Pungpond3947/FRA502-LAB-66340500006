# 3R Robot Control & Visualization (ROS 2)

This project implements a control system for a 3-DOF (Revolute-Revolute-Revolute) Robot Arm using ROS 2. It features Jacobian-based teleoperation, Inverse Kinematics (IK) trajectory planning, workspace visualization, and an automated task scheduler.

## Dependencies

Ensure you have the following installed in your ROS 2 environment:

* **ROS 2** (Humble/Foxy/Jazzy)
* **Robotics Toolbox for Python**: `pip3 install roboticstoolbox-python`
* **Spatial Math**: `pip3 install spatialmath-python`
* **NumPy**: `pip3 install numpy`

> **Note:** This package requires a custom interface package named `lab4_interfaces` containing the following services:
> * `SetMode.srv` (Request: `string mode`, `geometry_msgs/Pose target_pose` | Response: `bool success`, `string message`, `float64[] configuration`)
> * `Random.srv` (Request: `string mode` | Response: `geometry_msgs/Point position`, `bool inprogress`)

## File Descriptions

### 1. `controller.py` (Main Controller)
The core node that handles the robot's kinematics and control logic.
* **Kinematics:** Uses MDH parameters to define the 3R robot.
* **Modes:**
    * `TELEOP`: Velocity control using the Jacobian Matrix ($J^{-1} \cdot v$). Includes **Singularity Avoidance** (stops and resets if determinant is too low).
    * `IPK` (Inverse Position Kinematics): Moves the robot to a target pose using a linear trajectory.
* **Outputs:** Publishes `/joint_states` and End-Effector pose.

### 2. `teleop.py` (Keyboard Input)
A keyboard interface for manual control.
* **Output:** `/cmd_vel` (Twist message).
* **Features:** Supports switching between **World Frame** and **End-Effector Frame**.

### 3. `scheduler.py` (Task Manager)
Acts as a high-level state machine using `MultiThreadedExecutor`.
* **Function:** Manages mode switching via the `set_mode` service.
* **Auto Mode:** Triggers a sequence: *Get Random Pose -> Move Robot -> Wait 10s -> Repeat*.

### 4. `random_pose.py` (Target Generator)
Generates valid random coordinates for the robot.
* **Validation:** Checks if the random point is reachable (within workspace radius and solvable via IK).
* **Service:** Provides the `random_server` service.

### 5. `work_space.py` (Visualization)
Visualizes the reachable workspace of the robot.
* **Method:** Monte Carlo simulation (Random Joint Configurations -> Forward Kinematics).
* **Output:** Publishes a `PointCloud2` message to `/workspace_points` for Rviz visualization.

## Installation

**Install dependencies**

```bash
pip3 install numpy==1.26.4
pip3 install roboticstoolbox-python
sudo apt install ros-humble-desktop-full
sudo apt install ros-dev-tools
sudo apt install ros-humble-robot-state-publisher
```

Go somewhere like your home directory and clone this package.

```bash
git clone https://github.com/Pungpond3947/FRA502-LAB-66340500006.git
cd FRA502-LAB-66340500006/
```
then build (inside FUN4)

```bash
colcon build && . install/setup.bash
```
Set up your environment by sourcing the following file.

```bash
echo "source ~/FUN4/install/setup.bash" >> ~/.bashrc
```

## How to Run

**Open 3 Terminals in order and leave them running so that all three can run simultaneously.**

**Terminal 1**
```bash
ros2 launch example_description simple_display.launch.py
```
**Terminal 2**
```bash
ros2 run example_description teleop.py 
```

## Teleop Controls

When running `teleop.py`, use the following keys:

| Key | Action |
|-----|--------|
| `w` | Move +X (Forward) |
| `s` | Move -X (Backward) |
| `a` | Move +Y (Left) |
| `d` | Move -Y (Right) |
| `q` | Move +Z (Up) |
| `e` | Move -Z (Down) |
| `f` | Toggle Frame (World ↔ End-Effector) |
| `Ctrl+C` | Quit |

## 📡 Service API (Scheduler)

You can control the system modes manually using ROS 2 service calls:

### 1. Enable Teleoperation Mode:
```bash
ros2 service call /set_mode lab4_interfaces/srv/SetMode "{mode: 'TELEOP'}"
```

### 2. Enable Auto Loop Mode:
```bash
ros2 service call /set_mode lab4_interfaces/srv/SetMode "{mode: 'AUTO'}"
```

### 3. Send Specific Target (IPK):
```bash
ros2 service call /set_mode lab4_interfaces/srv/SetMode "{mode: 'IPK', target_pose: {position: {x: 0.2, y: 0.0, z: 0.2}}}"
```

## Technical Details

* **Singularity Handling:** The controller checks `det(J)`. If `abs(det_J) < 0.0005`, the robot stops and resets to a safe "home" configuration to prevent undefined behavior.

* **Frame Handling:** In Teleop, `twist.angular.z` is repurposed as a flag:
  * `0.0` for World Frame
  * `1.0` for End-Effector Frame
