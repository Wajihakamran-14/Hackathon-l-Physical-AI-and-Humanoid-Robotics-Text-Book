# Chapter 2: ROS 2 Nodes, Topics, & Services Explained

## Introduction

In the previous chapter, we had a brief overview of ROS 2 and its core components. In this chapter, we will take a deeper dive into three of the most fundamental concepts in ROS 2: Nodes, Topics, and Services. Understanding these concepts is crucial for building any ROS 2 application.

## Nodes

A ROS 2 system is a distributed network of processes called nodes. Each node is a process that performs some computation. A well-designed ROS 2 system is comprised of many nodes, each with a specific purpose. For example, you might have a node that controls a camera, a node that processes the camera's images, and another node that controls the robot's wheels based on the processed image data.

### Creating a Node

As we saw in the previous chapter, creating a node in Python is straightforward using `rclpy`.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

The `rclpy.spin()` function enters a loop that keeps the node running and allows it to process callbacks for things like topic messages and service requests.

## Topics

Topics are the primary way that nodes communicate with each other. A topic is a named bus over which nodes can send and receive messages. Topics use a publish/subscribe model. A node can publish messages to a topic, and any number of nodes can subscribe to that topic to receive the messages.

### Publishers and Subscribers

Let's expand on our example from the previous chapter. We'll create a publisher and a subscriber in two separate nodes.

**Publisher (`publisher_node.py`):**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from publisher: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = MyPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber (`subscriber_node.py`):**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber = MySubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Don't forget to update `setup.py` to include the new entry points.

You can run these nodes in separate terminals to see them communicating.

### Custom Messages

While ROS 2 provides a rich set of standard message types, you will often need to define your own custom messages. A message definition is a `.msg` file that defines the data structure of the message.

For example, let's create a `Num.msg` file in a new directory `msg` inside our package:

`my_first_ros_package/msg/Num.msg`:
```
int64 num
```

We need to tell ROS 2 to build our new message type. Modify `CMakeLists.txt` and `package.xml` in your package.

In `package.xml` add:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

In `CMakeLists.txt` (if it exists, if not, this is for C++ packages, ament_python is different) and for python packages you have to modify `setup.py` and `package.xml`. For `ament_python`, you have to modify `package.xml` as above and also ensure that your `setup.py` can find the message files. However, with modern `ament_cmake` and `colcon`, just having the files in the right place and the dependencies in `package.xml` is often enough for python. For C++ you would add:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
)
```

After building your workspace again with `colcon build`, you can use your custom message in your Python nodes:

```python
from my_first_ros_package.msg import Num
```

## Services

Services provide a request/reply model of communication. One node acts as a service server, and another node acts as a service client. The client sends a request to the server, and the server sends a response back to the client.

### Creating a Service Server and Client

Let's create a simple service that adds two integers.

First, we need a service definition file. Create a file `AddTwoInts.srv` in a new `srv` directory.

`my_first_ros_package/srv/AddTwoInts.srv`:
```
int64 a
int64 b
---
int64 sum
```

The part above `---` is the request, and the part below is the response.

Remember to add the necessary dependencies to `package.xml` for `rosidl_default_generators` and update your build files as you did for messages.

**Service Server (`service_server.py`):**
```python
from my_first_ros_package.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client (`service_client.py`):**
```python
import sys
from my_first_ros_package.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Run the server first, then the client with two numbers as arguments:
`ros2 run my_first_ros_package service_client 10 20`

## Summary

In this chapter, we explored ROS 2 Nodes, Topics, and Services in more detail. You learned how to create publishers, subscribers, service servers, and service clients. You also learned how to define your own custom messages and services. These are the building blocks you will use to create complex robotics applications. In the next chapter, we'll look at how to integrate Python agents with ROS 2.