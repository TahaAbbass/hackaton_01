---
title: Voice Processing with OpenAI Whisper
sidebar_label: Voice Processing with Whisper
description: Learn to implement voice-to-text processing using OpenAI Whisper for humanoid robotics applications
---

# Voice Processing with OpenAI Whisper

## Overview

OpenAI Whisper is a state-of-the-art speech recognition model that enables accurate voice-to-text conversion for humanoid robotics applications. This chapter covers implementing Whisper for voice-controlled robotics systems.

## Key Concepts

Whisper provides several key capabilities relevant to robotics:

### Speech Recognition Fundamentals
- Automatic speech recognition (ASR) for converting audio to text
- Multi-language support for international robotics applications
- Robust performance in various acoustic environments
- Real-time and batch processing capabilities

### Audio Preprocessing
Proper audio preprocessing is crucial for optimal Whisper performance:
- Noise reduction and filtering
- Audio normalization and amplification
- Format conversion (sampling rate, bit depth)
- Silence trimming and segmentation

### Integration with Robotics Systems
Whisper can be integrated with robotic systems through:
- Real-time audio streaming from robot microphones
- Batch processing of recorded audio segments
- Integration with ROS 2 audio topics and messages
- Voice command validation and safety checks

## Practical Implementation

Here's how to implement Whisper for voice processing in robotics:

### Installation and Setup

```bash
pip install openai-whisper
# Alternative: for GPU acceleration
pip install openai-whisper[cuda]
```

### Basic Whisper Integration

```python
import whisper
import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import io
import wave
import numpy as np

class VoiceProcessingNode:
    def __init__(self):
        rospy.init_node('voice_processing_node')

        # Load Whisper model (choose appropriate size for your needs)
        self.model = whisper.load_model("base")  # or "small", "medium", "large"

        # Audio subscription
        self.audio_sub = rospy.Subscriber('/audio_input', AudioData, self.audio_callback)

        # Voice command publisher
        self.voice_pub = rospy.Publisher('/voice_commands', String, queue_size=10)

        rospy.loginfo("Voice processing node initialized")

    def audio_callback(self, audio_msg):
        """Process incoming audio data with Whisper."""
        try:
            # Convert audio data to format suitable for Whisper
            audio_np = self.convert_audio_data(audio_msg)

            # Process with Whisper
            result = self.model.transcribe(audio_np)
            text = result['text'].strip()

            if text:  # Only publish if we got a transcription
                rospy.loginfo(f"Transcribed: {text}")

                # Create and publish voice command
                cmd_msg = String()
                cmd_msg.data = text
                self.voice_pub.publish(cmd_msg)

        except Exception as e:
            rospy.logerr(f"Error processing audio: {e}")

    def convert_audio_data(self, audio_msg):
        """Convert ROS AudioData message to numpy array for Whisper."""
        # Convert the raw audio bytes to a numpy array
        audio_bytes = np.frombuffer(audio_msg.data, dtype=np.int16)

        # Normalize to float32 in range [-1, 1]
        audio_float = audio_bytes.astype(np.float32) / 32768.0

        return audio_float

def main():
    try:
        node = VoiceProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
```

### Advanced Voice Processing Features

#### Voice Activity Detection (VAD)

For more efficient processing, implement voice activity detection:

```python
import webrtcvad
import collections

class VoiceActivityDetector:
    def __init__(self, sample_rate=16000, frame_duration_ms=30):
        self.vad = webrtcvad.Vad(2)  # Aggressiveness level 2
        self.sample_rate = sample_rate
        self.frame_duration_ms = frame_duration_ms
        self.frame_size = int(sample_rate * frame_duration_ms / 1000)

        # Ring buffer for storing audio frames
        self.ring_buffer = collections.deque(maxlen=30)  # Store last 30 frames

    def is_speech(self, audio_frame):
        """Detect if the audio frame contains speech."""
        return self.vad.is_speech(audio_frame, self.sample_rate)

    def detect_speech_activity(self, audio_data):
        """Detect speech activity in a chunk of audio data."""
        # Split audio into frames
        frames = self.split_audio_into_frames(audio_data, self.frame_size)

        speech_frames = 0
        for frame in frames:
            if self.is_speech(frame):
                speech_frames += 1

        # Return True if more than 30% of frames contain speech
        return speech_frames / len(frames) > 0.3 if len(frames) > 0 else False

    def split_audio_into_frames(self, audio_data, frame_size):
        """Split audio data into frames for VAD."""
        frames = []
        for i in range(0, len(audio_data), frame_size):
            frame = audio_data[i:i + frame_size]
            if len(frame) < frame_size:
                # Pad with zeros if frame is shorter than expected
                frame = np.pad(frame, (0, frame_size - len(frame)), 'constant')
            frames.append(frame.tobytes())
        return frames
```

#### Speaker Identification

For multi-user scenarios, implement speaker identification:

```python
from sklearn.mixture import GaussianMixture
import librosa
import numpy as np

class SpeakerIdentifier:
    def __init__(self):
        self.enrolled_speakers = {}  # Dictionary of speaker models
        self.speaker_names = []      # List of enrolled speaker names

    def extract_voice_features(self, audio_data, sr=16000):
        """Extract MFCC features from audio for speaker identification."""
        # Compute MFCCs
        mfccs = librosa.feature.mfcc(y=audio_data, sr=sr, n_mfcc=13)

        # Compute mean and variance of MFCCs
        features = np.hstack([
            np.mean(mfccs, axis=1),
            np.var(mfccs, axis=1)
        ])

        return features

    def enroll_speaker(self, audio_data, speaker_name):
        """Enroll a new speaker with their voice samples."""
        features = self.extract_voice_features(audio_data)

        # Create a GMM model for the speaker
        gmm = GaussianMixture(n_components=2, random_state=42)
        gmm.fit(features.reshape(1, -1))

        self.enrolled_speakers[speaker_name] = gmm
        self.speaker_names.append(speaker_name)

        rospy.loginfo(f"Speaker {speaker_name} enrolled successfully")

    def identify_speaker(self, audio_data):
        """Identify the speaker from audio data."""
        if not self.enrolled_speakers:
            return "unknown"

        features = self.extract_voice_features(audio_data)

        best_match = "unknown"
        best_score = float('-inf')

        for name, model in self.enrolled_speakers.items():
            try:
                score = model.score(features.reshape(1, -1))
                if score > best_score:
                    best_score = score
                    best_match = name
            except:
                continue

        return best_match
```

## Voice Command Processing

Once we have the transcribed text, we need to process it into actionable commands:

```python
class VoiceCommandProcessor:
    def __init__(self):
        # Define command patterns and their corresponding actions
        self.command_patterns = {
            r'go to (?P<location>\w+)': 'navigate_to_location',
            r'move (?P<direction>\w+)': 'move_direction',
            r'pick up (?P<object>.+)': 'pickup_object',
            r'wave|greet': 'wave_gesture',
            r'follow me': 'follow_user',
            r'stop|halt': 'stop_movement',
            r'help': 'provide_assistance'
        }

        # Compile regex patterns for efficiency
        self.compiled_patterns = {
            re.compile(pattern, re.IGNORECASE): action
            for pattern, action in self.command_patterns.items()
        }

    def process_command(self, text):
        """Process transcribed text and extract command and parameters."""
        text_lower = text.lower().strip()

        for pattern, action in self.compiled_patterns.items():
            match = pattern.search(text_lower)
            if match:
                params = match.groupdict()
                return action, params

        # If no pattern matches, return generic command
        return 'unknown_command', {'text': text}

    def execute_command(self, action, params):
        """Execute the identified command."""
        # This would interface with the robot's action system
        # Implementation would depend on specific robot capabilities
        rospy.loginfo(f"Executing command: {action} with params: {params}")

        # Example command execution
        if action == 'navigate_to_location':
            self.navigate_to(params.get('location'))
        elif action == 'move_direction':
            self.move_in_direction(params.get('direction'))
        elif action == 'wave_gesture':
            self.perform_wave_gesture()
        # ... other actions

    def navigate_to(self, location):
        """Navigate to a specified location."""
        # Implementation would use navigation stack
        pass

    def move_in_direction(self, direction):
        """Move in a specified direction."""
        # Implementation would send velocity commands
        pass

    def perform_wave_gesture(self):
        """Perform waving gesture."""
        # Implementation would control robot arm joints
        pass
```

## Hands-on Exercise: Creating a Voice Command Recognition System

In this exercise, you'll create a complete voice command recognition system:

1. Set up Whisper for voice-to-text conversion
2. Implement voice activity detection
3. Create command pattern matching
4. Test the system with various voice commands

### Exercise Steps:

1. Install the required dependencies:
   ```bash
   pip install openai-whisper webrtcvad scikit-learn librosa
   ```

2. Create the voice processing node as shown in the examples above

3. Test with various voice commands like "move forward", "go to kitchen", "wave hello"

4. Validate that the system correctly transcribes and processes voice commands

## Validation Techniques

To ensure your voice processing system works correctly:

1. **Accuracy Testing**: Compare Whisper transcriptions with ground truth
2. **Latency Measurement**: Measure time from audio input to text output
3. **Robustness Testing**: Test with various noise levels and accents
4. **Integration Testing**: Verify the system works with your robot's command interface

## Summary

Voice processing with OpenAI Whisper enables natural human-robot interaction for humanoid robots. By properly implementing audio preprocessing, speech recognition, and command processing, you can create intuitive voice-controlled robotic systems.

## Next Steps

In the next chapter, we'll explore LLM-based cognitive planning for converting natural language commands into ROS 2 actions.