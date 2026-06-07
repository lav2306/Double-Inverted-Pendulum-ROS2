# Double Inverted Pendulum — ROS + Gazebo (LQR Control)

A simulation of a **Double Inverted Pendulum** built with ROS Noetic and Gazebo Classic.  
The system uses a pre-computed **LQR (Linear Quadratic Regulator)** controller to balance two rods upright on a fixed base.

---

## System Overview

```
        ● bob_link (red sphere, 0.1 kg)
        |
        | rod2_link (green, 0.8 m, 0.3 kg)
        |
     [joint2] ← effort controlled
        |
        | rod1_link (blue, 0.8 m, 0.5 kg)
        |
     [joint1] ← effort controlled
        |
   [base_link] (grey cart, 2.0 kg, fixed to world)
```

**Goal:** Hold both rods vertically upright (θ1 = 0, θ2 = 0) using torque commands computed by the LQR controller.

---

## Control Law

The system is linearised around the upright equilibrium. The LQR gain matrix **K** (2×4) maps the state vector to joint torques:

```
u = -K * x
```

where:
- `x = [θ1, θ2, θ1_dot, θ2_dot]` — joint angles and velocities
- `u = [u1, u2]` — torques applied at joint1 and joint2

**LQR Gains (pre-computed):**
```
K = [[95.0,  160.0,  22.0,  18.0],   # joint1 torque
     [45.0,  120.0,  10.0,  15.0]]   # joint2 torque
```

Computed for: m1=0.5 kg, L1=0.8 m, m2=0.3 kg, L2=0.8 m, M=2.0 kg, g=9.81 m/s²

Torque is saturated at ±150 N·m.

---

## Robot Parameters

| Component | Parameter | Value |
|-----------|-----------|-------|
| Base (cart) | Mass | 2.0 kg |
| Rod 1 (lower) | Mass | 0.5 kg |
| Rod 1 | Length | 0.8 m |
| Rod 2 (upper) | Mass | 0.3 kg |
| Rod 2 | Length | 0.8 m |
| Tip bob | Mass | 0.1 kg |
| Joint damping | Both joints | 0.005 |
| Joint limits | Both joints | ±90° |

---

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/double_inv_pend/joint_states` | `sensor_msgs/JointState` | Joint positions and velocities (subscribed) |
| `/double_inv_pend/joint1_effort_controller/command` | `std_msgs/Float64` | Torque command for joint1 (published) |
| `/double_inv_pend/joint2_effort_controller/command` | `std_msgs/Float64` | Torque command for joint2 (published) |

---

## Project Structure

```
double_inv_pend/
├── urdf/
│   └── double_inv_pend.urdf      # Robot description
├── launch/
│   └── double_inv_pend.launch    # Launches Gazebo + controllers
├── config/
│   └── controllers.yaml          # ros_control configuration
├── control/
│   └── control.py                # LQR controller node
├── Dockerfile                    # Docker environment (ROS Noetic)
├── CMakeLists.txt
└── package.xml
```

---

## Requirements

- Docker (recommended) **or** Ubuntu 20.04 with ROS Noetic
- Gazebo Classic (comes with `ros-noetic-desktop-full`)
- `ros-noetic-effort-controllers`
- `ros-noetic-ros-controllers`
- `ros-noetic-gazebo-ros-control`

---

## Running with Docker (Recommended)

### 1. Build and start the container

```bash
xhost +local:docker

docker run -it \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd):/root/catkin_ws/src/double_inv_pend" \
  --name inv_pend_sim \
  osrf/ros:noetic-desktop-full \
  bash
```

### 2. Inside the container — install dependencies and build

```bash
apt-get update && apt-get install -y \
  ros-noetic-effort-controllers \
  ros-noetic-ros-controllers \
  ros-noetic-gazebo-ros-control

source /opt/ros/noetic/setup.bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Terminal 1 — ROS Master

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 4. Terminal 2 — Gazebo Simulation

```bash
docker exec -it inv_pend_sim bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
roslaunch double_inv_pend double_inv_pend.launch
```

### 5. Terminal 3 — LQR Controller

```bash
docker exec -it inv_pend_sim bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
rosrun double_inv_pend control.py
```

---

## Tuning the LQR Gains

Gains can be overridden at runtime via ROS parameters without recompiling:

```bash
rosrun double_inv_pend control.py \
  _k1_th1:=95.0 _k1_th2:=160.0 _k1_dth1:=22.0 _k1_dth2:=18.0 \
  _k2_th1:=45.0 _k2_th2:=120.0 _k2_dth1:=10.0 _k2_dth2:=15.0
```

---

## Dependencies

```xml
rospy, std_msgs, sensor_msgs,
controller_manager, effort_controllers,
joint_state_controller, robot_state_publisher,
gazebo_ros, gazebo_ros_control
```

---

## Authors
LAV KUMAR SAINI, PARTH DEGWEKAR, PRANAY TRIVEDI, HASHEM AL ATTAS
Robotics & Automation Engineering — Symbiosis Institute of Technology, Pune
