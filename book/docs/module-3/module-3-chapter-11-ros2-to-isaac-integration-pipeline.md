# Chapter 11: ROS 2 → Isaac Integration Pipeline

## Introduction

We've discussed Isaac Sim and ROS 2 as powerful but separate tools. The true power emerges when they are seamlessly integrated. Isaac Sim has built-in support for ROS 2, allowing for a bidirectional flow of information. This chapter provides a practical guide to setting up the integration pipeline, enabling you to send sensor data from Isaac Sim to ROS 2 and command the robot in Isaac Sim from ROS 2.

## The ROS 2 Bridge in Isaac Sim

Isaac Sim's ROS 2 integration is handled by the "ROS 2 Bridge." This is a component within Isaac Sim that can be enabled and configured to bridge data between the Isaac Sim environment and the ROS 2 network.

Key features of the ROS 2 Bridge:
-   **Bidirectional Communication:** It can both publish data from Isaac Sim to ROS 2 topics and subscribe to ROS 2 topics to control objects in the simulation.
-   **Wide Range of Message Types:** It supports a large number of standard ROS message types out-of-the-box (e.g., `sensor_msgs/Image`, `sensor_msgs/LaserScan`, `geometry_msgs/Twist`, `tf2_msgs/TFMessage`).
-   **Configurable:** You can specify which topics to bridge and the direction of the bridge (Isaac Sim to ROS, or ROS to Isaac Sim).
-   **Python API:** The bridge can be configured and controlled using Isaac Sim's Python API.

## Setting up the Pipeline

Here’s a step-by-step guide to the integration pipeline.

### 1. Enable the ROS 2 Bridge Extension

In Isaac Sim, features are provided through "Extensions." First, you need to ensure the `omni.isaac.ros2_bridge` extension is enabled. You can do this through the UI (Window -> Extensions) or in your Python script.

### 2. Create the Robot and Sensors in Isaac Sim

Using the Python API or the UI, you load your robot's USD or URDF file into the scene. You then add the sensors you need (cameras, LiDAR, IMU, etc.) to the robot model. As you create these sensors, you'll give their topics names (e.g., `/rgb_camera/image_raw`).

### 3. Configure the Bridge in Python

The most flexible way to configure the bridge is through a Python script. You can specify which topics to publish from Isaac Sim and which to subscribe to.

```python
from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
import omni.isaac.ros2_bridge as ros2_bridge

# Assuming a world is created
world = World()
# Add your robot and sensors...

# Get the ROS 2 bridge interface
ros2_bridge_interface = ros2_bridge.get_ros2_bridge_interface()

# --- Publishing from Isaac Sim to ROS 2 ---

# Bridge camera data
ros2_bridge_interface.add_ros2_publisher("rgb_camera/image_raw", "sensor_msgs/msg/Image")
ros2_bridge_interface.add_ros2_publisher("rgb_camera/camera_info", "sensor_msgs/msg/CameraInfo")

# Bridge LiDAR data
ros2_bridge_interface.add_ros2_publisher("lidar/scan", "sensor_msgs/msg/LaserScan")

# Bridge transform data (tf)
ros2_bridge_interface.add_ros2_publisher("tf", "tf2_msgs/msg/TFMessage")


# --- Subscribing from ROS 2 to Isaac Sim ---

# Bridge velocity commands to control the robot
# This assumes the robot has a "DifferentialBase" controller set up in Isaac Sim
ros2_bridge_interface.add_ros2_subscriber("cmd_vel", "geometry_msgs/msg/Twist")

# You can also bridge joint commands
ros2_bridge_interface.add_ros2_subscriber(
    "joint_trajectory_controller/joint_trajectory",
    "trajectory_msgs/msg/JointTrajectory"
)

# After setting up the bridges, you would start the simulation
world.play()

# Your script would then run in a loop
while world.is_playing():
    world.step(render=True)

world.stop()
```

### 4. Run Your ROS 2 Nodes

With Isaac Sim running and publishing sensor data, you can now run your ROS 2 nodes in a separate terminal (or as part of a ROS 2 launch file). These nodes will receive the data from Isaac Sim as if it were a real robot.

For example, you could run:
-   `rviz2` to visualize the sensor data and robot model.
-   `isaac_ros_vslam` to process the stereo camera data.
-   `nav2` for navigation.

### 5. Closing the Loop: Control

To control the robot, your ROS 2 nodes (e.g., Nav2's controller server) will publish messages (like a `Twist` on `/cmd_vel`). The ROS 2 Bridge in Isaac Sim subscribes to this topic, receives the message, and applies the corresponding velocity to the simulated robot.

This creates a complete "hardware-in-the-loop"-style simulation, where your entire ROS 2 software stack is running as it would on the real robot, but it's interacting with a high-fidelity simulated robot and environment.

## Summary

This chapter detailed the practical steps for setting up an integration pipeline between ROS 2 and Isaac Sim. We covered how to use the ROS 2 Bridge to publish sensor data from the simulation and subscribe to ROS 2 topics to control the simulated robot. This powerful pipeline is fundamental for developing and testing complex robotics applications like navigation and manipulation in a realistic, safe, and efficient manner. This concludes Module 3. In the final module, we will explore how to add high-level intelligence to our robot using voice and large language models.