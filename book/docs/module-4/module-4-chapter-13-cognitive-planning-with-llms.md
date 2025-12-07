# Chapter 13: Cognitive Planning with LLMs

## Introduction

The previous chapter used simple keyword spotting to understand user commands. This approach is brittle and doesn't scale to complex, multi-step instructions. Large Language Models (LLMs) like GPT-4 offer a much more powerful and flexible way to understand natural language and reason about the world. In this chapter, we'll explore how to use LLMs for "cognitive planning," where the LLM acts as the robot's "brain," breaking down high-level goals into sequences of executable actions.

## LLMs as Robot Brains

LLMs have shown remarkable abilities in reasoning, planning, and code generation. We can leverage these abilities to build more intelligent robots. The core idea is to treat the LLM as a "planner" that takes a high-level goal (e.g., "clean up the table") and outputs a sequence of low-level commands that the robot can execute (e.g., `navigateTo('table')`, `detectObjects()`, `pickUp('apple')`, `placeIn('bin')`).

## A Simple LLM-based Planning Architecture

A simple architecture for an LLM-powered robot might look like this:

1.  **Prompt Engineering:** We create a carefully crafted prompt that gives the LLM the context it needs to act as a robot planner. This prompt includes:
    -   The robot's overall goal (e.g., "You are a helpful home assistant robot.").
    -   A description of the available actions (the "action space"). This is like an API documentation for the robot.
    -   Information about the current state of the world (e.g., "You are in the kitchen. You see a table with an apple on it.").
    -   The user's command.

2.  **LLM Call:** We send this prompt to the LLM.

3.  **Parsing the Response:** The LLM's response will be a sequence of actions in a format we've specified (e.g., JSON or a list of function calls). We need to parse this response.

4.  **Action Execution:** We execute the actions one by one, using the action handlers we developed in the previous chapter.

5.  **Feedback Loop:** After each action, we can update the world state and even feed the result back into the LLM to allow it to replan if something went wrong.

### Example Prompt

Here's an example of a prompt for an LLM-based planner:

```
You are a helpful home assistant robot. Your goal is to follow user commands.

You have the following actions available to you:
- `navigateTo(location)`: Moves the robot to a specified location. Locations can be: 'kitchen', 'living_room', 'table'.
- `detectObjects()`: Returns a list of objects you see.
- `pickUp(object)`: Picks up a specified object. The object must be visible.
- `placeIn(location)`: Places the object you are holding in a specified location.

Current state: You are in the living room. You see a 'table'.

User command: "Can you please take the apple from the table and throw it away in the kitchen?"

Your plan is:
```

We would expect the LLM to respond with a plan like:
```json
[
  {"action": "navigateTo", "parameters": ["table"]},
  {"action": "detectObjects", "parameters": []},
  {"action": "pickUp", "parameters": ["apple"]},
  {"action": "navigateTo", "parameters": ["kitchen"]},
  {"action": "placeIn", "parameters": ["trash_bin"]}
]
```

## Creating a ROS 2 Cognitive Planner Node

We can encapsulate this logic in a "cognitive planner" ROS 2 node.

This node would:
1.  **Subscribe to the `/transcribed_text` topic** to get the user's command.
2.  **Maintain World State:** It would subscribe to other topics (e.g., from a perception system) to maintain a simplified understanding of the world state (robot location, visible objects).
3.  **Construct the Prompt:** When a command is received, it constructs the prompt with the robot's persona, action space, current state, and the user's command.
4.  **Call the LLM API:** It makes a call to an LLM API (like OpenAI's) with the prompt.
5.  **Parse and Publish Actions:** It parses the LLM's response and publishes the sequence of actions on the `/robot_action` topic we defined in the previous chapter. The action handlers would then execute them sequentially.

## Challenges and Considerations

While powerful, using LLMs for robotics has challenges:
-   **Latency:** Calling an external LLM API can be slow. For real-time interaction, you might need to use smaller, locally-run models or techniques to speed up inference.
-   **Hallucination:** LLMs can sometimes "hallucinate" or generate actions that are not possible or not in the action space. The prompt needs to be very clear, and the parser needs to be robust to invalid responses.
-   **Grounding:** The LLM's understanding of the world is purely text-based. "Grounding" the LLM's concepts (like "table") to the robot's perceptual reality (the point cloud and image data that represents the table) is a major research area.
-   **Safety:** You must have a safety layer that prevents the LLM from generating and executing dangerous actions. The action handlers should validate actions before execution.

## Summary

This chapter introduced the exciting concept of using Large Language Models for cognitive planning in robotics. We outlined an architecture where an LLM, guided by a carefully engineered prompt, acts as a planner, breaking down high-level user goals into low-level robot actions. While there are challenges to address, this approach represents a significant step towards creating more intelligent, flexible, and capable robots that can understand and respond to us in a more natural way. In the final chapter, we'll bring everything together in a capstone project.