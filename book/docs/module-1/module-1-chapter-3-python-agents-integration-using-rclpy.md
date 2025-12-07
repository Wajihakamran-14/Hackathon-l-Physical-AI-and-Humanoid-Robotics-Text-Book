# Chapter 3: Python Agents Integration using rclpy

## Introduction

In the previous chapters, we've learned the fundamentals of ROS 2. Now, let's explore how to integrate existing Python code and logic into the ROS 2 ecosystem. We'll refer to these encapsulated pieces of logic as "Python Agents." An agent could be anything from a simple algorithm to a complex machine learning model. The goal is to wrap these agents in a ROS 2 node, allowing them to communicate with other parts of a robotic system.

## What is a Python Agent?

For our purposes, a Python Agent is a class or module that encapsulates a specific functionality. It has inputs, outputs, and some internal state. For example, an agent could be:

-   A computer vision model that takes an image and outputs object detections.
-   A path planning algorithm that takes a map and a goal, and outputs a path.
-   A state machine that manages the robot's behavior.

By wrapping these agents in ROS 2 nodes, we can create modular and reusable robotics applications.

## Integrating a Simple Agent

Let's consider a simple agent that performs a "computation". This agent will take a number, perform a calculation, and return the result. We want to expose this agent's functionality through a ROS 2 service.

Here's our simple agent class:

**`my_agent.py`**
```python
class MyAgent:
    def __init__(self):
        self._internal_state = 0

    def process_data(self, input_data):
        # A simple computation
        self._internal_state += 1
        return input_data * 2 + self._internal_state
```

Now, let's create a ROS 2 node that wraps this agent and exposes its functionality through a service. We will use the `AddTwoInts.srv` definition from the previous chapter for simplicity, but imagine it's a more descriptive service like `ProcessData.srv`.

**`agent_node.py`**
```python
import rclpy
from rclpy.node import Node
from my_first_ros_package.srv import AddTwoInts # Re-using for demonstration
from .my_agent import MyAgent

class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')
        self.agent = MyAgent()
        self.srv = self.create_service(AddTwoInts, 'process_data', self.process_data_callback)

    def process_data_callback(self, request, response):
        # We'll just use one of the inputs for this example
        input_data = request.a
        self.get_logger().info(f'Agent node received: {input_data}')
        
        # Use the agent to process the data
        result = self.agent.process_data(input_data)
        
        response.sum = result # Using the 'sum' field of the response
        self.get_logger().info(f'Agent processed data, result: {result}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    agent_node = AgentNode()
    rclpy.spin(agent_node)
    agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
In this example, the `AgentNode` creates an instance of `MyAgent`. When the `process_data` service is called, the node passes the data to the agent, gets the result, and sends it back as the service response.

## Agent with Topic-based Communication

Another common pattern is to have an agent that subscribes to a topic, processes the data, and publishes the result on another topic. This is useful for continuous data streams, like processing sensor data.

Let's imagine our agent now processes a stream of strings.

**`streaming_agent.py`**
```python
class StreamingAgent:
    def __init__(self):
        pass

    def process_string(self, input_string):
        return f"Processed: {input_string.upper()}"
```

Now the ROS 2 wrapper node:

**`streaming_agent_node.py`**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .streaming_agent import StreamingAgent

class StreamingAgentNode(Node):
    def __init__(self):
        super().__init__('streaming_agent_node')
        self.agent = StreamingAgent()
        
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'output_topic', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received on input_topic: "{msg.data}"')
        
        # Process the data with the agent
        result = self.agent.process_string(msg.data)
        
        # Publish the result
        output_msg = String()
        output_msg.data = result
        self.publisher.publish(output_msg)
        self.get_logger().info(f'Published to output_topic: "{output_msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    streaming_agent_node = StreamingAgentNode()
    rclpy.spin(streaming_agent_node)
    streaming_agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

You can test this node by publishing to `input_topic` and listening to `output_topic`:

```bash
# Terminal 1
ros2 run my_first_ros_package streaming_agent_node

# Terminal 2
ros2 topic pub /input_topic std_msgs/msg/String "data: 'hello agent'"

# Terminal 3
ros2 topic echo /output_topic
```

## Summary

In this chapter, we've seen how to integrate Python code, which we've called "agents," into a ROS 2 system. By wrapping our logic in ROS 2 nodes, we can leverage the power of the ROS 2 communication system (topics, services, etc.) to build modular and scalable robotics applications. This is a powerful pattern that you will use frequently when building complex robots. In the next chapter, we will look at how to describe the physical structure of a robot using URDF.