---
title: LLM Cognitive Planning for Robotics
sidebar_label: LLM Cognitive Planning
description: Learn to use Large Language Models for cognitive planning to convert natural language commands into ROS 2 actions
---

# LLM Cognitive Planning for Robotics

## Overview

Large Language Models (LLMs) provide powerful capabilities for cognitive planning in robotics, enabling robots to understand natural language commands and convert them into executable ROS 2 actions. This chapter covers implementing LLM-based cognitive planning for humanoid robots.

## Key Concepts

LLM cognitive planning involves several key components:

### Natural Language Understanding (NLU)
- Intent recognition from natural language commands
- Entity extraction for objects, locations, and parameters
- Context awareness for multi-turn conversations
- Ambiguity resolution for unclear commands

### Action Sequence Generation
- Mapping high-level intentions to low-level robot actions
- Generating appropriate ROS 2 action servers and clients
- Sequencing actions for complex multi-step tasks
- Handling conditional execution based on sensor feedback

### Context Management
- Maintaining conversation state across multiple commands
- Remembering previous actions and their outcomes
- Managing robot state and available capabilities
- Handling interruptions and priority changes

## LLM Integration for Robotics

### Selecting the Right LLM

For robotics applications, consider these factors when selecting an LLM:

- **Response Speed**: Critical for real-time interaction
- **Reliability**: Consistent output for safety-critical applications
- **Customization**: Ability to fine-tune for robotics vocabulary
- **Cost**: Consider token usage for frequent interactions

### Prompt Engineering for Robotics

Effective prompt engineering is crucial for reliable LLM performance in robotics:

```python
class LLMPromptEngineer:
    def __init__(self):
        self.system_prompt = """
        You are a cognitive planning assistant for a humanoid robot. Your job is to convert natural language commands into structured robot actions.

        Available actions:
        - move_to(location): Move the robot to a specific location
        - pickup(object): Pick up an object
        - place(object, location): Place an object at a location
        - wave(): Wave to greet someone
        - speak(text): Speak text aloud
        - look_at(target): Turn head to look at target
        - follow(person): Follow a person
        - stop(): Stop current action

        Output format: JSON with "action" and "parameters" keys.
        Example: {{"action": "move_to", "parameters": {{"location": "kitchen"}}}
        """

    def create_task_prompt(self, user_command, robot_state, available_actions):
        """Create a prompt for a specific task."""
        return f"""
        {self.system_prompt}

        Current robot state: {robot_state}
        Available actions: {available_actions}
        User command: "{user_command}"

        Respond with a JSON object containing the action and parameters.
        """

    def parse_llm_response(self, response_text):
        """Parse and validate the LLM response."""
        try:
            # Extract JSON from response (in case it includes additional text)
            json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
            if json_match:
                json_str = json_match.group()
                parsed = json.loads(json_str)

                # Validate required keys
                if 'action' in parsed:
                    return parsed
                else:
                    raise ValueError("Missing 'action' in response")
            else:
                raise ValueError("No JSON found in response")
        except json.JSONDecodeError:
            raise ValueError(f"Invalid JSON response: {response_text}")
        except Exception as e:
            raise ValueError(f"Error parsing response: {e}")
```

### Safety and Validation

LLM outputs must be validated before execution:

```python
class SafetyValidator:
    def __init__(self):
        # Define safe locations and actions
        self.safe_locations = ["kitchen", "living_room", "bedroom", "office", "hallway"]
        self.safe_actions = [
            "move_to", "pickup", "place", "wave", "speak",
            "look_at", "follow", "stop", "take_picture"
        ]

        # Define dangerous actions to block
        self.dangerous_keywords = [
            "self-destruct", "damage", "harm", "break", "unsafe",
            "forbidden", "restricted", "dangerous"
        ]

    def validate_action(self, action_dict):
        """Validate an action for safety and appropriateness."""
        action = action_dict.get('action', '')
        params = action_dict.get('parameters', {})

        # Check if action is safe
        if action not in self.safe_actions:
            return False, f"Action '{action}' is not allowed"

        # Check if location is safe (for movement actions)
        if action == 'move_to':
            location = params.get('location', '').lower()
            if location not in self.safe_locations:
                return False, f"Location '{location}' is not safe to navigate to"

        # Check for dangerous keywords
        for param_value in params.values():
            if isinstance(param_value, str):
                for keyword in self.dangerous_keywords:
                    if keyword.lower() in param_value.lower():
                        return False, f"Command contains dangerous keyword: {keyword}"

        # Additional safety checks based on robot capabilities
        if action == 'pickup' and 'object' in params:
            obj = params['object'].lower()
            if obj in ['human', 'person', 'pet']:
                return False, f"Cannot pickup living beings: {obj}"

        return True, "Action is safe to execute"
```

## Implementation Example

Here's a complete implementation of an LLM-based cognitive planning system:

```python
import openai
import rospy
import json
import re
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class LLMBasedCognitivePlanner:
    def __init__(self):
        rospy.init_node('llm_cognitive_planner')

        # Initialize OpenAI client
        self.client = openai.OpenAI(api_key=rospy.get_param('~openai_api_key'))

        # System prompt for robotics cognitive planning
        self.system_prompt = """
        You are a cognitive planning assistant for a humanoid robot. Convert natural language commands into structured robot actions.

        Available actions:
        - move_to(location): Navigate to a location (locations: kitchen, living_room, bedroom, office, hallway, dining_room)
        - pickup(object): Pick up an object (objects: cup, book, ball, toy, bottle)
        - place(object, destination): Place an object somewhere
        - wave(): Wave to greet
        - speak(text): Say something aloud
        - look_at(target): Turn head toward target
        - follow(person): Follow a person
        - take_picture(): Take a photo
        - stop(): Stop current action

        Output format: {{"action": "...", "parameters": {{"...": "..."}}}
        Example: {{"action": "move_to", "parameters": {{"location": "kitchen"}}}
        """

        # Subscriptions and publications
        self.voice_cmd_sub = rospy.Subscriber(
            '/voice_commands',
            String,
            self.voice_command_callback
        )

        self.action_status_pub = rospy.Publisher(
            '/cognitive_planner/status',
            String,
            queue_size=10
        )

        # Robot state
        self.current_location = "home_base"
        self.carried_object = None
        self.is_moving = False

        # Action clients
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("LLM Cognitive Planner initialized")

    def voice_command_callback(self, msg):
        """Process voice command using LLM cognitive planning."""
        command = msg.data
        rospy.loginfo(f"Received voice command: {command}")

        try:
            # Generate plan using LLM
            plan = self.generate_plan_with_llm(command)

            if plan:
                # Validate the plan for safety
                is_safe, reason = self.validate_action(plan)

                if is_safe:
                    # Execute the planned action
                    success = self.execute_action(plan)

                    if success:
                        status_msg = String()
                        status_msg.data = f"Successfully executed: {plan['action']}"
                        self.action_status_pub.publish(status_msg)
                    else:
                        status_msg = String()
                        status_msg.data = f"Failed to execute: {plan['action']}"
                        self.action_status_pub.publish(status_msg)
                else:
                    rospy.logerr(f"Unsafe action blocked: {reason}")
                    status_msg = String()
                    status_msg.data = f"Blocked unsafe action: {reason}"
                    self.action_status_pub.publish(status_msg)
            else:
                rospy.logwarn(f"Could not generate plan for command: {command}")

        except Exception as e:
            rospy.logerr(f"Error processing command with LLM: {e}")
            status_msg = String()
            status_msg.data = f"Error processing command: {e}"
            self.action_status_pub.publish(status_msg)

    def generate_plan_with_llm(self, command):
        """Generate a plan using LLM cognitive planning."""
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {
                        "role": "user",
                        "content": f"Current location: {self.current_location}\nCarried object: {self.carried_object}\nUser command: '{command}'"
                    }
                ],
                max_tokens=200,
                temperature=0.1  # Low temperature for more consistent responses
            )

            llm_response = response.choices[0].message.content

            # Parse the LLM response
            plan = self.parse_llm_response(llm_response)

            rospy.loginfo(f"Generated plan: {plan}")
            return plan

        except Exception as e:
            rospy.logerr(f"Error generating plan with LLM: {e}")
            return None

    def parse_llm_response(self, response_text):
        """Parse and validate the LLM response."""
        try:
            # Extract JSON from response
            json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
            if json_match:
                json_str = json_match.group()
                parsed = json.loads(json_str)

                if 'action' in parsed:
                    return parsed
                else:
                    raise ValueError("Missing 'action' in response")
            else:
                raise ValueError("No JSON found in response")
        except json.JSONDecodeError:
            raise ValueError(f"Invalid JSON response: {response_text}")
        except Exception as e:
            raise ValueError(f"Error parsing response: {e}")

    def validate_action(self, action_dict):
        """Validate an action for safety and appropriateness."""
        action = action_dict.get('action', '')
        params = action_dict.get('parameters', {})

        # Check if action is safe
        safe_actions = [
            "move_to", "pickup", "place", "wave", "speak",
            "look_at", "follow", "take_picture", "stop"
        ]

        if action not in safe_actions:
            return False, f"Action '{action}' is not allowed"

        # Check if location is safe (for movement actions)
        safe_locations = ["kitchen", "living_room", "bedroom", "office", "hallway", "dining_room"]
        if action == 'move_to':
            location = params.get('location', '').lower()
            if location not in safe_locations:
                return False, f"Location '{location}' is not safe to navigate to"

        # Additional safety checks
        if action == 'pickup' and 'object' in params:
            obj = params['object'].lower()
            forbidden_objects = ['human', 'person', 'pet', 'living_being']
            if obj in forbidden_objects:
                return False, f"Cannot pickup living beings: {obj}"

        return True, "Action is safe to execute"

    def execute_action(self, plan):
        """Execute the planned action."""
        action = plan.get('action', '')
        params = plan.get('parameters', {})

        rospy.loginfo(f"Executing action: {action} with params: {params}")

        if action == 'move_to':
            return self.execute_move_to(params.get('location'))
        elif action == 'wave':
            return self.execute_wave()
        elif action == 'speak':
            return self.execute_speak(params.get('text', ''))
        elif action == 'look_at':
            return self.execute_look_at(params.get('target', ''))
        elif action == 'stop':
            return self.execute_stop()
        elif action == 'take_picture':
            return self.execute_take_picture()
        else:
            rospy.logwarn(f"Unsupported action: {action}")
            return False

    def execute_move_to(self, location):
        """Execute move to location action."""
        if not self.move_base_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Move base server not available")
            return False

        # Convert location to coordinates (in a real system, you'd have a map)
        # This is a simplified example
        coordinates = self.get_coordinates_for_location(location)
        if coordinates is None:
            rospy.logerr(f"Unknown location: {location}")
            return False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = coordinates[0]
        goal.target_pose.pose.position.y = coordinates[1]
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal)

        # Wait for result with timeout
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(30.0))

        if not finished_within_time:
            rospy.logerr("Move base action timed out")
            self.move_base_client.cancel_goal()
            return False

        state = self.move_base_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            self.current_location = location
            rospy.loginfo(f"Successfully moved to {location}")
            return True
        else:
            rospy.logerr(f"Failed to move to {location}, state: {state}")
            return False

    def get_coordinates_for_location(self, location):
        """Get coordinates for a named location."""
        # In a real system, this would come from a map or localization system
        location_map = {
            "kitchen": (3.0, 1.0),
            "living_room": (0.0, 0.0),
            "bedroom": (-2.0, 1.5),
            "office": (1.5, -2.0),
            "hallway": (0.0, 1.0),
            "dining_room": (2.0, 2.0)
        }
        return location_map.get(location.lower())

    def execute_wave(self):
        """Execute waving action."""
        # In a real system, this would control robot arm joints
        rospy.loginfo("Executing wave action")
        # Simulate the action
        rospy.sleep(2.0)
        return True

    def execute_speak(self, text):
        """Execute speech action."""
        # In a real system, this would use text-to-speech
        rospy.loginfo(f"Speaking: {text}")
        # Simulate the action
        rospy.sleep(len(text.split()) * 0.5)  # Approximate speaking time
        return True

    def execute_look_at(self, target):
        """Execute look at action."""
        rospy.loginfo(f"Looking at: {target}")
        # In a real system, this would control head/neck joints
        # Simulate the action
        rospy.sleep(1.0)
        return True

    def execute_stop(self):
        """Execute stop action."""
        # Cancel any active goals
        if self.move_base_client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            self.move_base_client.cancel_all_goals()

        rospy.loginfo("Stopped all actions")
        return True

    def execute_take_picture(self):
        """Execute picture taking action."""
        rospy.loginfo("Taking picture")
        # In a real system, this would trigger camera capture
        # Simulate the action
        rospy.sleep(1.0)
        return True


def main():
    try:
        planner = LLMBasedCognitivePlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
```

## Advanced Cognitive Planning Techniques

### Multi-Step Task Planning

For complex commands, implement multi-step planning:

```python
class MultiStepPlanner:
    def __init__(self):
        self.action_sequences = {
            "bring_me_coffee": [
                {"action": "move_to", "parameters": {"location": "kitchen"}},
                {"action": "pickup", "parameters": {"object": "coffee_cup"}},
                {"action": "move_to", "parameters": {"location": "living_room"}},
                {"action": "place", "parameters": {"object": "coffee_cup", "destination": "table"}}
            ],
            "greet_and_tell_joke": [
                {"action": "wave"},
                {"action": "speak", "parameters": {"text": "Hello! Here's a joke: Why did the robot go on vacation? Because it needed to recharge!"}}
            ]
        }

    def plan_complex_task(self, task_name):
        """Plan a complex multi-step task."""
        return self.action_sequences.get(task_name, [])
```

### Context-Aware Planning

Consider the robot's current state when planning:

```python
class ContextAwarePlanner:
    def __init__(self):
        self.robot_state = {
            'location': 'home_base',
            'carried_object': None,
            'battery_level': 100,
            'is_busy': False
        }

    def update_robot_state(self, new_state):
        """Update the robot's current state."""
        self.robot_state.update(new_state)

    def contextual_plan(self, command):
        """Generate plan considering current context."""
        # Modify planning based on current state
        if self.robot_state['battery_level'] < 20:
            if 'move_to' in command.lower():
                # Suggest charging instead
                return {"action": "move_to", "parameters": {"location": "charging_station"}}

        if self.robot_state['is_busy']:
            return {"action": "speak", "parameters": {"text": "I'm currently busy. Please wait."}}

        # Otherwise, proceed with normal planning
        return self.normal_plan(command)
```

## Hands-on Exercise: Implementing LLM-Based Cognitive Planning

In this exercise, you'll create a complete LLM-based cognitive planning system:

1. Set up OpenAI API integration
2. Implement prompt engineering for robotics commands
3. Create safety validation mechanisms
4. Test the system with various natural language commands

### Exercise Steps:

1. Obtain an OpenAI API key and configure it for your system
2. Implement the cognitive planner as shown in the examples
3. Test with commands like "Go to the kitchen", "Wave hello", "Take a picture"
4. Validate that the system correctly interprets commands and executes appropriate actions

## Validation Techniques

To ensure your LLM cognitive planning system works correctly:

1. **Command Interpretation Testing**: Verify various natural language expressions map to correct actions
2. **Safety Validation**: Test that dangerous commands are properly blocked
3. **Context Awareness**: Verify the system considers robot state when planning
4. **Error Handling**: Test responses to ambiguous or invalid commands

## Summary

LLM-based cognitive planning enables robots to understand and execute complex natural language commands. By properly implementing prompt engineering, safety validation, and context awareness, you can create intelligent robots that respond naturally to human instructions.

## Next Steps

In the final chapter, we'll combine voice processing and LLM cognitive planning in a complete capstone project demonstrating autonomous humanoid behavior through voice commands.