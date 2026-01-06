# Chapter 12: Voice-to-Action Pipeline (OpenAI Whisper)

## Introduction

To create truly interactive and intelligent robots, we need more natural ways to communicate with them than a keyboard or a joystick. Voice is one of the most intuitive interfaces for humans. In this chapter, we'll build a "voice-to-action" pipeline that allows a user to give spoken commands to a robot. We'll use OpenAI's Whisper model for speech-to-text and then process the text to generate a robot action.

## The Voice-to-Action Pipeline

Our pipeline will have three main stages:
1.  **Speech-to-Text:** Convert spoken audio into a text transcript.
2.  **Text-to-Intent:** Analyze the text to understand the user's intention.
3.  **Intent-to-Action:** Map the user's intent to a specific robot action.

## 1. Speech-to-Text with OpenAI Whisper

Whisper is a powerful, open-source automatic speech recognition (ASR) model from OpenAI. It is trained on a large and diverse dataset of audio and is highly robust to background noise, accents, and different languages.

### Setting up Whisper

You can use Whisper through OpenAI's API or by running the model locally. For a robotics application where real-time response is important and internet connectivity may not be guaranteed, running it locally is often preferred.

You can install the Python package for Whisper:
```bash
pip install -U openai-whisper
```

### Creating a ROS 2 Node for Whisper

We can create a ROS 2 node that listens for audio data, transcribes it using Whisper, and publishes the resulting text.

The node would:
1.  **Subscribe to an audio topic:** This topic would carry audio data, perhaps of type `audio_common_msgs/msg/AudioData`. You'd need a separate node to handle the microphone input and publish on this topic.
2.  **Buffer the audio:** Collect audio data until a period of silence is detected, indicating the user has finished speaking.
3.  **Transcribe with Whisper:** Once a complete utterance is captured, pass it to the Whisper model for transcription.
4.  **Publish the text:** Publish the transcribed text on a new topic, e.g., `/transcribed_text` (of type `std_msgs/msg/String`).

Here's a conceptual Python script for the Whisper node:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# Assume an audio message type and a VAD (Voice Activity Detection) utility
import whisper

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.subscription = self.create_subscription(
            AudioData, 'audio', self.audio_callback, 10)
        self.publisher = self.create_publisher(String, 'transcribed_text', 10)
        self.model = whisper.load_model("base.en") # Choose model size
        self.audio_buffer = []

    def audio_callback(self, msg):
        # Add audio to buffer
        # Use a VAD to detect end of speech
        # ...
        
        # If end of speech detected:
        # 1. Convert audio buffer to a numpy array
        audio_np = self.convert_buffer_to_numpy(self.audio_buffer)
        
        # 2. Transcribe
        result = self.model.transcribe(audio_np)
        text = result['text']
        self.get_logger().info(f'Transcribed: "{text}"')
        
        # 3. Publish
        text_msg = String()
        text_msg.data = text
        self.publisher.publish(text_msg)
        
        # 4. Clear buffer
        self.audio_buffer = []

# ... main function ...
```

## 2. Text-to-Intent

Once we have the transcribed text, we need to understand what the user wants. This is a Natural Language Understanding (NLU) problem. For a limited set of commands, we can use simple keyword spotting. For more complex commands, a more sophisticated NLU model would be needed.

Let's stick with keyword spotting for now. We can create a "command processor" node that subscribes to `/transcribed_text`.

```python
# In a new CommandProcessorNode

def text_callback(self, msg):
    text = msg.data.lower()
    
    if "go to the kitchen" in text:
        self.publish_action("navigate", "kitchen")
    elif "pick up the red cube" in text:
        self.publish_action("pick", "red_cube")
    # ... more commands
    
def publish_action(self, action_type, target):
    # Publish a custom message, e.g., of type RobotAction.msg
    action_msg = RobotAction()
    action_msg.type = action_type
    action_msg.target = target
    self.action_publisher.publish(action_msg)
```

## 3. Intent-to-Action

The final step is to execute the action. We would have different "action handler" nodes responsible for different types of actions.

-   **Navigation Action Handler:**
    -   Subscribes to the `/robot_action` topic.
    -   If it sees a "navigate" action, it looks up the coordinates for the target (e.g., "kitchen") from a database.
    -   It then uses a ROS 2 action client to send a goal to the Nav2 stack.

-   **Manipulation Action Handler:**
    -   Subscribes to the `/robot_action` topic.
    -   If it sees a "pick" action, it would initiate a perception and manipulation pipeline (e.g., find the "red_cube", plan a path for the arm, and grasp it).

## Summary

In this chapter, we've designed a voice-to-action pipeline that enables natural language communication with a robot. We used OpenAI's Whisper for robust speech-to-text, a simple keyword-based approach for intent recognition, and a set of action handlers to execute the commands. This architecture provides a flexible way to add voice control to a ROS 2-based robot. In the next chapter, we'll explore how to make the intent recognition stage much more powerful using Large Language Models.