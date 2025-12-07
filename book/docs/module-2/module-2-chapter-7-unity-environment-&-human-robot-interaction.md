# Chapter 7: Unity Environment & Human-Robot Interaction

## Introduction

While Gazebo is a powerful physics-based simulator, the Unity game engine has emerged as a compelling alternative for robotics simulation, especially when high-fidelity graphics, complex human-robot interaction (HRI), and virtual reality (VR) are required. In this chapter, we'll introduce how Unity can be used for robotics simulation and its integration with ROS 2.

## Why Unity for Robotics?

Unity offers several advantages for robotics simulation:
- **High-Fidelity Graphics:** Unity is a professional game engine known for its stunning visual quality. This is crucial for training and testing vision-based algorithms and for creating realistic environments for HRI studies.
- **Rich Asset Store:** the Unity Asset Store provides a vast library of 3D models, environments, and tools that can be used to rapidly build complex simulation worlds.
- **C# Scripting:** Unity uses C# for scripting, which is a powerful, modern object-oriented language.
- **Strong VR/AR Support:** Unity is a leader in the VR/AR space, making it an excellent choice for developing applications that involve immersive teleoperation or interaction with virtual robots.

## Unity Robotics Hub

To facilitate the use of Unity for robotics, Unity has developed the "Unity Robotics Hub." This is a set of packages that provide ROS 2 integration, URDF import, and other robotics-specific features.

The key packages are:
- **ROS TCP Connector:** This package handles the communication between Unity and ROS 2. It sets up a TCP endpoint that ROS 2 nodes can connect to, sending and receiving ROS messages.
- **URDF Importer:** This tool allows you to import a URDF file into Unity, automatically creating the robot's visual and collision geometry and setting up the joint hierarchy.
- **Visualizations:** The hub includes tools for visualizing sensor data (like laser scans and trajectories) directly within the Unity environment.

## Setting up a Unity Project with ROS 2

1.  **Install Unity:** Download and install Unity Hub and a recent version of the Unity Editor.
2.  **Create a Project:** Create a new 3D project in Unity.
3.  **Install Robotics Packages:** Using the Unity Package Manager, install the ROS TCP Connector and other packages from the Robotics Hub. You will typically do this by adding packages from a git URL.
4.  **Configure ROS Connection:** In your Unity project, you'll configure the ROS TCP Connector with the IP address of your ROS 2 machine.

## Example: Controlling a Robot in Unity from ROS 2

Let's say you have imported your robot's URDF into Unity. The joints can be controlled by a C# script. To connect this to ROS, you would:

1.  **Create a ROS Publisher in Python:** Write a ROS 2 node that publishes joint commands (e.g., of type `sensor_msgs/msg/JointState`) to a topic.

2.  **Create a C# Subscriber in Unity:** Write a C# script in Unity that subscribes to the same topic using the ROS TCP Connector. This script will receive the `JointState` messages.

3.  **Apply Joint Commands:** The C# script will then take the received joint positions and apply them to the robot's joints in the Unity simulation. Unity's `ArticulationBody` component is often used for physics-based joint control.

A simplified C# subscriber might look like this:
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class MyRobotController : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("joint_commands", OnJointCommand);
    }

    void OnJointCommand(JointStateMsg msg)
    {
        // Code to parse the message and apply joint positions
        // to the robot's ArticulationBody components.
        Debug.Log("Received joint command");
    }
}
```

## Human-Robot Interaction (HRI) in Unity

Unity's strengths really shine in HRI scenarios. You can:
- **Create virtual humans:** Use animated human models to test how a robot interacts with people.
- **Use VR for teleoperation:** Create a VR interface that allows a human to control the robot immersively. The user can see through the robot's "eyes" and control its limbs with VR controllers.
- **Simulate complex social environments:** Build realistic homes, hospitals, or public spaces to test a robot's ability to navigate and interact in human-centric environments.

## Summary

This chapter provided an overview of using the Unity game engine for robotics simulation. We discussed the advantages of Unity, the tools available in the Unity Robotics Hub, and how to connect a Unity simulation to a ROS 2 system. For applications requiring high-fidelity graphics, VR, or complex human interaction, Unity is an excellent tool to have in your robotics development toolbox. This concludes Module 2 on simulation. In the next module, we'll start to look at more advanced topics in robot perception and navigation.