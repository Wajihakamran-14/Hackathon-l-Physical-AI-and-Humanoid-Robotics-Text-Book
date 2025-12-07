# Chapter 4: URDF for Humanoids & Launch Files

## Introduction

So far, we have focused on the software and communication aspects of ROS 2. However, a robot is a physical entity. To work with a robot in simulation and for many real-world tasks like motion planning, we need a way to describe the robot's physical structure. This is where the Unified Robot Description Format (URDF) comes in. In this chapter, we will also learn about ROS 2 launch files, which allow us to run complex systems with multiple nodes.

## What is URDF?

URDF is an XML format used in ROS to describe all elements of a robot. These include:

-   **Links:** The rigid parts of the robot (e.g., the body, an arm segment). Each link has properties like mass, inertia, and a visual representation (how it looks) and a collision model (for physics simulation).
-   **Joints:** The connections between links. Joints define how links can move relative to each other. Common joint types are `revolute` (for rotating joints), `prismatic` (for sliding joints), and `fixed` (for rigid connections).
-   **Sensors and Actuators:** While not directly part of the core URDF spec for kinematics and dynamics, URDF can be extended using tags (like in SDF) to include sensor and actuator information, especially for simulation in Gazebo.

### A Simple URDF Example

Let's create a simple URDF for a two-link arm.

**`two_link_arm.urdf`**
```xml
<?xml version="1.0"?>
<robot name="two_link_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <link name="link1">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="base_to_link1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.025"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>

  <link name="link2">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="link1_to_link2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.5 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>

</robot>
```

This URDF describes a robot with a base, and two links connected by revolute joints.

### Visualizing URDF

ROS 2 provides tools to visualize a URDF file. The `robot_state_publisher` node reads the URDF and publishes the state of the robot's transformations to the `/tf` topic. RViz2 can then be used to visualize the robot model.

To do this, you would typically have a launch file that starts `robot_state_publisher` and `joint_state_publisher_gui` (to move the joints with sliders).

## ROS 2 Launch Files

As your robotics systems become more complex, you'll find yourself needing to run many nodes at once. A ROS 2 launch file is a script that can start and configure multiple nodes. Launch files can be written in Python, XML, or YAML. Python launch files are the most powerful and flexible.

### A Simple Python Launch File

Let's create a launch file that starts our publisher and subscriber nodes from Chapter 2.

Create a `launch` directory in your package. Inside, create `pub_sub_launch.py`:

**`my_first_ros_package/launch/pub_sub_launch.py`**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_ros_package',
            executable='my_publisher_node', # The name from your setup.py entry_points
            name='publisher'
        ),
        Node(
            package='my_first_ros_package',
            executable='my_subscriber_node', # The name from your setup.py entry_points
            name='subscriber'
        ),
    ])
```
You'll need to update your `setup.py` to install the launch file.
```python
# in setup.py
import os
from glob import glob
...
setup(
    ...
    data_files=[
        ...
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    ...
)
```

After rebuilding your workspace, you can run your launch file:

```bash
ros2 launch my_first_ros_package pub_sub_launch.py
```

This will start both the publisher and subscriber nodes.

### Launching a URDF-based Robot

A more complex launch file is needed to launch a robot with a URDF. This typically involves:
1.  Finding the URDF file.
2.  Starting `robot_state_publisher` with the URDF content.
3.  Starting `joint_state_publisher` or `joint_state_publisher_gui`.
4.  Starting RViz2 with a specific configuration.

Here's an example:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file_name = 'two_link_arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('my_first_ros_package'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])
```
(This assumes you have placed your URDF in a `urdf` directory and installed it via `setup.py`).

## Summary

In this chapter, we learned about URDF for describing a robot's structure and ROS 2 launch files for managing complex applications. These are essential tools for both simulation and working with real hardware. With these four chapters, you now have a solid foundation in the core concepts of ROS 2. The next modules will build on this foundation to explore more advanced topics in robotics.