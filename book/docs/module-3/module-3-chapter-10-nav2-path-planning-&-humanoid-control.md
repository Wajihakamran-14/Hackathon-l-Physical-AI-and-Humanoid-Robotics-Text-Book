# Chapter 10: Nav2 Path Planning & Humanoid Control

## Introduction

Nav2 is the second generation of the ROS Navigation Stack. It's a powerful and flexible framework for autonomous navigation, providing everything from global path planning to local obstacle avoidance and robot control. While often associated with wheeled robots, Nav2 can be adapted for more complex robots like humanoids. This chapter explores the architecture of Nav2 and discusses the challenges and strategies for using it to control a humanoid robot.

## The Nav2 Architecture

Nav2 is not a single node but a collection of servers, lifecycle nodes, and plugins. The main components are:

-   **BT Navigator:** The top-level executive that orchestrates the navigation process using a Behavior Tree. It can call on the planner, controller, and recovery servers.
-   **Planner Server:** Responsible for finding a valid path from the robot's current position to a goal position in the global map. It uses a plugin-based architecture, so you can choose from different global planners (e.g., A*, D*, SmacPlanner).
-   **Controller Server:** Responsible for generating velocity commands to follow the global path while avoiding local obstacles. Like the planner, it is plugin-based, with common choices being DWB (Dynamic Window Approach) and TEB (Timed Elastic Band).
-   **Recovery Server:** Holds a set of recovery behaviors to handle situations where the robot gets stuck (e.g., clearing the costmap, spinning in place).
-   **Costmaps:** Nav2 uses two costmaps:
    -   **Global Costmap:** A 2D map of the environment used by the global planner.
    -   **Local Costmap:** A smaller, rolling window around the robot used by the local planner for dynamic obstacle avoidance.

## Path Planning with Nav2

The general workflow for path planning in Nav2 is:
1.  A goal is sent to the BT Navigator (e.g., from RViz2 or a ROS 2 action client).
2.  The BT Navigator calls the Planner Server to generate a global plan.
3.  The Planner Server uses a global planning algorithm (like A*) to find a path in the global costmap.
4.  The BT Navigator then passes this path to the Controller Server.
5.  The Controller Server generates velocity commands (`Twist` messages) to follow the path, using the local costmap to avoid immediate obstacles.
6.  These `Twist` messages are published on the `/cmd_vel` topic.

## Challenges of Humanoid Control

Applying Nav2 to a humanoid robot is not straightforward. The output of Nav2 is a `Twist` message (`geometry_msgs/msg/Twist`), which specifies linear velocity (forward/backward, side-to-side) and angular velocity (turning). A simple wheeled robot can directly translate these commands into wheel speeds. A humanoid is much more complex.

The key challenges are:
-   **Balance and Stability:** A humanoid robot must actively balance itself while walking. A simple velocity command is not enough; you need a controller that can maintain stability.
-   **Gait Generation:** Humanoids have different gaits (walking, running, turning in place). A gait generator is needed to translate high-level commands into the complex sequence of joint movements required for walking.
-   **Whole-Body Control:** To move effectively, a humanoid must coordinate its entire body, including arms for balance and a torso for stability.

## Bridging Nav2 to a Humanoid

To use Nav2 with a humanoid, you need a "humanoid controller" node that sits between Nav2's `/cmd_vel` output and the robot's joint controllers.

This humanoid controller node would:
1.  **Subscribe to `/cmd_vel`:** Receive the desired linear and angular velocities from Nav2.
2.  **Translate `Twist` to Gait Parameters:** Convert the `Twist` message into parameters for a gait generator. For example:
    -   `linear.x` > 0 -> Walk forward with a certain speed.
    -   `linear.x` < 0 -> Walk backward.
    -   `angular.z` != 0 -> Turn while walking or turn in place.
3.  **Use a Gait Generator:** A gait generator (often based on a model like the inverted pendulum model) produces a sequence of desired foot placements and center of mass trajectories.
4.  **Inverse Kinematics:** An inverse kinematics solver calculates the joint angles required to achieve the desired foot placements and body posture.
5.  **Publish Joint Commands:** The calculated joint angles are published to the robot's joint controllers (e.g., on a `/joint_trajectory_controller/joint_trajectory` topic).

This is a complex control problem, and there are many different approaches to humanoid locomotion. Some research platforms use advanced techniques like model predictive control (MPC) to generate dynamic and robust walking behaviors.

## Summary

In this chapter, we've explored the architecture of the Nav2 stack and its core components for path planning and control. We then delved into the specific challenges of applying Nav2 to humanoid robots, highlighting the need for an intermediate humanoid controller to translate high-level velocity commands into stable walking gaits. While challenging, integrating Nav2 with a sophisticated whole-body controller is a key step towards achieving autonomous navigation for a humanoid robot. The next chapter will focus on the practicalities of integrating ROS 2 with Isaac Sim.