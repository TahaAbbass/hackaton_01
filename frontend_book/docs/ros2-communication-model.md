---
title: ROS 2 Communication Model
sidebar_label: Communication Model
description: Understanding Nodes, Topics, Services, and agent ↔ controller flow in ROS 2
---

# ROS 2 Communication Model

## Overview of Communication Concepts

ROS 2 provides several communication patterns to enable different types of interactions between nodes:

- **Nodes**: The fundamental unit of computation in ROS 2
- **Topics**: Asynchronous, many-to-many communication using publish/subscribe pattern
- **Services**: Synchronous, request/response communication
- **Actions**: Long-running tasks with feedback and goal management

## Nodes in ROS 2

Nodes are the basic execution units of a ROS 2 program. Each node runs a specific task and communicates with other nodes through:

- Publishing/subscribing to topics
- Making requests to services
- Providing services to other nodes
- Executing or requesting actions

### Creating a Node in Python (rclpy)

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
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
```

## Topics and Message Passing

Topics enable asynchronous communication through a publish/subscribe model:

- Publishers send messages to topics
- Subscribers receive messages from topics
- Multiple publishers and subscribers can exist for the same topic
- Communication is decoupled in time and space

### Example Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AgentPublisher(Node):
    def __init__(self):
        super().__init__('agent_publisher')
        self.publisher_ = self.create_publisher(String, 'agent_commands', 10)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published command: {command}')
```

### Example Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControllerSubscriber(Node):
    def __init__(self):
        super().__init__('controller_subscriber')
        self.subscription = self.create_subscription(
            String,
            'agent_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # Process the command and take appropriate action
```

## Services and Request/Response Patterns

Services provide synchronous communication with request/response semantics:

- A client sends a request to a service
- The service processes the request and returns a response
- This pattern is useful for actions that require confirmation or results

### Example Service Server

```python
from rclpy.node import Node
from rclpy.qos import QoSProfile
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

### Example Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
```

## rclpy-based Agent ↔ Controller Flow

In humanoid robotics, agents and controllers often communicate through ROS 2 topics and services. Here's a complete example of agent-controller communication:

### Agent Node (Decision Maker)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class HumanoidAgent(Node):
    def __init__(self):
        super().__init__('humanoid_agent')

        # Publisher for movement commands
        self.cmd_publisher = self.create_publisher(String, 'movement_commands', 10)

        # Subscriber for sensor data
        self.sensor_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for decision making
        self.timer = self.create_timer(1.0, self.make_decision)

    def joint_state_callback(self, msg):
        # Process sensor data
        self.get_logger().info(f'Received joint states: {len(msg.name)} joints')

    def make_decision(self):
        # Simple decision logic
        command = "move_forward"
        msg = String()
        msg.data = command
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f'Agent decision: {command}')
```

### Controller Node (Actuator)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Subscriber for commands from agent
        self.cmd_subscriber = self.create_subscription(
            String,
            'movement_commands',
            self.command_callback,
            10
        )

        # Publisher for joint trajectories
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10
        )

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Convert high-level command to joint trajectories
        trajectory = self.generate_trajectory_for_command(command)
        self.trajectory_publisher.publish(trajectory)

    def generate_trajectory_for_command(self, command):
        # Implementation to generate joint trajectories
        trajectory = JointTrajectory()
        # ... trajectory generation logic
        return trajectory
```

## Practical Exercises

1. **Create a Simple Publisher-Subscriber Pair**: Implement a publisher that sends "Hello, ROS 2!" every second and a subscriber that prints the received message.

2. **Build an Agent-Controller System**: Create a simple agent that decides to move forward or backward based on a simulated sensor reading, and a controller that executes the movement.

3. **Implement a Service**: Create a service that calculates the optimal walking gait based on terrain data.

In the next chapter, we'll explore how to describe robot structure using URDF.