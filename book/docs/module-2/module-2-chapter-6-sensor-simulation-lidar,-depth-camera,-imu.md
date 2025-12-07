# Chapter 6: Sensor Simulation (LiDAR, Depth Camera, IMU)

## Introduction

Sensors are the robot's eyes and ears, providing the data needed to perceive and interact with the world. In simulation, we need to model these sensors to test our perception, navigation, and control algorithms. Gazebo provides a rich set of sensor plugins that can be added to a URDF or SDF file to simulate various sensors. In this chapter, we'll focus on three common sensors: LiDAR, depth cameras, and IMUs.

## Gazebo Sensor Plugins

Gazebo uses plugins to model sensors. A plugin is a shared library that is loaded at runtime. Gazebo has a set of built-in plugins for common sensors. To use a sensor plugin, you add a `<sensor>` tag to your robot's SDF/URDF file, and within that, a `<plugin>` tag.

The general structure looks like this:

```xml
<gazebo reference="link_where_sensor_is_attached">
  <sensor type="sensor_type" name="my_sensor">
    ... sensor-specific parameters ...
    <plugin name="my_plugin_name" filename="libplugin_name.so">
      ... plugin-specific parameters ...
    </plugin>
  </sensor>
</gazebo>
```

Let's look at how to do this for our three sensors. We will be using the `ros_gz_bridge` to bridge the Gazebo topics to ROS 2 topics.

### 1. LiDAR (Laser Scanner)

A LiDAR (Light Detection and Ranging) sensor measures distances by illuminating a target with a laser and measuring the reflected light. It's commonly used for mapping and obstacle avoidance.

Here's how to add a LiDAR sensor to a URDF:

```xml
<gazebo reference="lidar_link">
  <sensor type="gpu_lidar" name="gpu_lidar_sensor">
    <topic>/lidar</topic>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin filename="libgz-sensors-gpu-lidar.so" name="gz::sim::systems::GpuLidar">
      <topic>/lidar</topic>
    </plugin>
  </sensor>
</gazebo>
```

The `ros_gz_bridge` can then be used to bridge the Gazebo topic `/lidar` (of type `gz.msgs.LaserScan`) to a ROS 2 topic `/lidar_ros` (of type `sensor_msgs/msg/LaserScan`).

### 2. Depth Camera

A depth camera provides a 2D image where each pixel's value represents the distance to the object at that point. They are essential for 3D perception and reconstruction.

Here's an example of a depth camera plugin:

```xml
<gazebo reference="camera_link">
  <sensor type="depth_camera" name="depth_camera_sensor">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R_FLOAT32</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <update_rate>30</update_rate>
    <plugin name="gz::sim::systems::DepthCamera" filename="libgz-sensors-depth-camera.so">
      <topic>/depth_camera</topic>
    </plugin>
  </sensor>
</gazebo>
```
The `ros_gz_bridge` will be needed to convert `gz.msgs.Image` to `sensor_msgs/msg/Image`.

### 3. IMU (Inertial Measurement Unit)

An IMU measures and reports a body's specific force, angular rate, and sometimes the orientation of the body, using a combination of accelerometers, gyroscopes, and sometimes magnetometers. IMUs are crucial for estimating a robot's orientation and state.

Adding an IMU sensor:
```xml
<gazebo reference="imu_link">
  <sensor name='imu_sensor' type='imu'>
    <update_rate>100</update_rate>
    <plugin filename="libgz-sensors-imu.so" name="gz::sim::systems::Imu">
      <topic>/imu</topic>
    </plugin>
  </sensor>
</gazebo>
```
The bridge will convert `gz.msgs.IMU` to `sensor_msgs/msg/Imu`.

## Using `ros_gz_bridge`

The `ros_gz_bridge` is a key tool for connecting Gazebo to ROS 2. It's a node that subscribes to a Gazebo topic, converts the message, and publishes it on a ROS 2 topic, and vice-versa.

You can run the bridge from the command line:
```bash
ros2 run ros_gz_bridge bridge <gz_topic>@<gz_type>[<ros2_topic>@<ros2_type>
```
For example, to bridge the LiDAR topic:
```bash
ros2 run ros_gz_bridge bridge /lidar@gz.msgs.LaserScan[sensor_msgs/msg/LaserScan@/lidar_ros
```

It's more common to include the bridge in your launch file:
```python
# In your launch file
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/lidar@gz.msgs.LaserScan[sensor_msgs/msg/LaserScan@/lidar_ros'],
    output='screen'
)
```

## Summary

In this chapter, you've learned how to add simulated sensors to a robot in Gazebo. We've covered how to add LiDAR, depth camera, and IMU sensors to a URDF file and how to use the `ros_gz_bridge` to make the sensor data available in ROS 2. With these simulated sensors, you can now start building and testing perception and navigation systems for your robot. The next chapter will explore using Unity as an alternative simulation environment.