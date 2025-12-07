# Chapter 5: Physics Simulation in Gazebo

## Introduction

Simulation is a critical tool in robotics. It allows for rapid testing of algorithms, safe experimentation, and development without access to physical hardware. Gazebo is a powerful, open-source 3D robotics simulator that works seamlessly with ROS. In this chapter, we'll learn how to use Gazebo to simulate a robot and its environment.

## What is Gazebo?

Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. At its core, Gazebo is a physics engine, and it also provides a rich set of sensor models and a graphical interface.

Key features of Gazebo:
- **Physics Engines:** Gazebo supports multiple physics engines, such as ODE, Bullet, Simbody, and DART.
- **Sensor Models:** A wide range of sensors can be simulated, including cameras, LiDAR, IMUs, GPS, and more.
- **Graphical Interface:** A GUI allows you to visualize and interact with the simulated world.
- **ROS Integration:** Gazebo is tightly integrated with ROS, allowing you to control your simulated robot using ROS nodes, topics, and services.

## URDF vs. SDF

While we used URDF in the previous module to describe our robot's kinematics, Gazebo uses the **Simulation Description Format (SDF)**. SDF is an XML format that is more expressive than URDF and is designed specifically for simulation. SDF can describe everything from robots and sensors to lighting and environmental properties.

Fortunately, you don't have to throw away your URDF files. ROS provides tools to convert URDF to SDF on the fly. You can also add Gazebo-specific tags to your URDF to specify things like colors, textures, and sensor plugins. These tags are wrapped in a `<gazebo>` block.

### Example: Adding a Gazebo color to a URDF
```xml
<link name="my_link">
  ...
</link>

<gazebo reference="my_link">
  <material>Gazebo/Red</material>
</gazebo>
```

## Creating a World

In Gazebo, the environment is described in a "world" file, which is an SDF file. A world file can contain models (like robots, tables, walls), lighting, and physics properties.

Here's a very simple world file that contains a ground plane and a sun:

**`my_world.world`**
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```
Gazebo comes with many pre-built models that you can include.

## Launching Gazebo with ROS 2

To launch Gazebo and spawn a robot in it, you'll typically use a ROS 2 launch file. The `ros_gz_sim` package provides the necessary tools.

Here's an example launch file that starts Gazebo with a specified world and spawns a robot from a URDF file.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to the world file
    world_path = os.path.join(get_package_share_directory('my_robot_pkg'), 'worlds', 'my_world.world')
    
    # Path to the URDF file
    urdf_file_name = 'my_robot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_pkg'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        # Start Gazebo server
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),

        # Start Gazebo GUI
        ExecuteProcess(
            cmd=['gz', 'sim', '-g'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # Spawn the robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', urdf_path, '-name', 'my_robot'],
            output='screen'
        )
    ])
```

This launch file does four things:
1.  Starts the Gazebo simulation server (`gz sim -r`). The `-r` flag tells it to run (i.e., start the physics simulation).
2.  Starts the Gazebo GUI (`gz sim -g`).
3.  Starts the `robot_state_publisher` to publish the robot's state to TF2.
4.  Uses the `ros_gz_sim`'s `create` executable to spawn the robot model into the simulation.

## Summary

In this chapter, we've introduced Gazebo as a powerful tool for robotics simulation. We discussed the difference between URDF and SDF and how to launch Gazebo and spawn a robot using a ROS 2 launch file. In the next chapter, we will look at how to add and simulate sensors in Gazebo.