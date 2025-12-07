# Chapter 1: Introduction to ROS 2 & Robot Control

## Introduction to ROS 2

Welcome to the world of robotics! This chapter introduces the Robot Operating System 2 (ROS 2), the foundational software framework we will use throughout this book. ROS 2 is an open-source, flexible framework for writing robot software. It is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS 2 has what you need for your next robotics project.

### Why ROS 2?

ROS 2 is an evolution of the original ROS framework, redesigned from the ground up to address the needs of modern robotics, including:

-   **Multi-robot systems:** ROS 2's underlying communication system (DDS) allows for robust communication between multiple robots.
-   **Real-time control:** ROS 2 provides support for real-time systems, which is critical for precise robot control.
-   **Support for small embedded systems:** ROS 2 can run on microcontrollers and other resource-constrained devices.
-   **Cross-platform compatibility:** ROS 2 supports Linux, macOS, and Windows.

## Core Concepts of ROS 2

ROS 2 applications are a graph of processes (called **nodes**) that communicate with each other. The main concepts are:

-   **Nodes:** A node is an executable that uses ROS 2 to communicate with other nodes. Each node in ROS should be responsible for a single, module purpose (e.g., one node for controlling wheel motors, one node for controlling a laser range-finder).
-   **Topics:** Topics are named buses over which nodes exchange messages. Topics have anonymous publish/subscribe semantics. This means that a node that publishes a message on a topic does not know which nodes will receive it.
-   **Messages:** Nodes communicate by passing messages. A message is a simple data structure, comprising typed fields. Standard primitive types (integer, floating point, boolean, etc.) are supported, as are arrays of primitive types.
-   **Services:** The publish/subscribe model is a very flexible communication paradigm, but its many-to-many one-way transport is not appropriate for request/reply interactions, which are often required in a distributed system. For this, ROS 2 provides Services.
-   **Actions:** Actions are for long-running tasks. They are similar to services, but they provide feedback on the task's progress and are preemptible.

## Your First ROS 2 Program

Let's write a simple "Hello World" program in ROS 2 using Python. This program will create a node that publishes a message to a topic.

First, make sure you have a ROS 2 workspace. If you don't, you can create one:

```bash
mkdir -p ros2_ws/src
cd ros2_ws
colcon build
```

Now, let's create a simple Python package and a node.

```bash
cd src
ros2 pkg create --build-type ament_python my_first_ros_package --dependencies rclpy
```

This will create a `my_first_ros_package` directory with a `setup.py`, `package.xml`, and other files.

Inside `my_first_ros_package/my_first_ros_package`, create a file named `my_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Now, we need to add an entry point to `setup.py` to make our node executable. Open `setup.py` and add the following to the `console_scripts` in the `entry_points`:

```python
# inside setup.py
...
    entry_points={
        'console_scripts': [
            'my_node = my_first_ros_package.my_node:main',
        ],
    },
...
```

Build the package:

```bash
cd ~/ros2_ws
colcon build
```

Source the setup file:

```bash
. install/setup.bash
```

Run the node:

```bash
ros2 run my_first_ros_package my_node
```

In another terminal, you can listen to the topic:

```bash
ros2 topic echo /topic
```

You should see the "Hello World" messages being published.

## Summary

In this chapter, we've had a brief introduction to ROS 2 and its core concepts. We've also written and run our first ROS 2 program. In the next chapter, we'll dive deeper into ROS 2 nodes, topics, and services.