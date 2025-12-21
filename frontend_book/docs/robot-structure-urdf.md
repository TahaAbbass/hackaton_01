---
title: Robot Structure with URDF
sidebar_label: Robot Structure with URDF
description: Understanding URDF for humanoid robot description and simulation readiness
---

# Robot Structure with URDF

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including:

- Links: Rigid parts of the robot
- Joints: Connections between links
- Visual and collision properties
- Inertial properties
- Transmission elements for actuators

## URDF Structure and XML Format

A complete URDF file follows this basic structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Links define the rigid parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

### Key URDF Elements

- `<robot>`: Root element that contains the entire robot description
- `<link>`: Represents a rigid body part of the robot
- `<joint>`: Defines the connection between two links
- `<visual>`: How the link appears in visualization
- `<collision>`: How the link interacts in physics simulation
- `<inertial>`: Mass and inertia properties for physics simulation

## Links, Joints, and Kinematics

### Links

Links represent rigid bodies in the robot. Each link can have:
- Visual properties (how it looks)
- Collision properties (how it interacts)
- Inertial properties (mass, center of mass, etc.)

```xml
<link name="upper_arm">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/upper_arm.stl"/>
    </geometry>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.2" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
  </inertial>
</link>
```

### Joints

Joints define how links connect and move relative to each other:

- `revolute`: Rotational joint with limits
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint with limits
- `fixed`: No movement between links
- `floating`: 6 DOF movement
- `planar`: Movement on a plane

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0.0 0.0 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="1.5" effort="30" velocity="1.0"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Kinematics

Kinematics describes the motion of the robot without considering forces. Forward kinematics calculates end-effector position from joint angles, while inverse kinematics calculates joint angles for a desired end-effector position.

## Visual and Collision Elements

### Visual Elements

Visual elements define how the robot appears in RViz and simulation:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.2 0.3"/>
    <!-- Other options: <sphere radius="0.1"/>, <cylinder radius="0.1" length="0.2"/>, <mesh filename="..."/> -->
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision Elements

Collision elements define how the robot interacts with the environment in simulation:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.2 0.3"/>
  </geometry>
</collision>
```

## Complete Humanoid Robot URDF Example

Here's a more comprehensive example of a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Left Lower Arm -->
  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Elbow Joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0.0 0.0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="15" velocity="1"/>
  </joint>
</robot>
```

## Simulation Readiness

For URDF to work properly in simulation:

1. **Physical Properties**: All links must have realistic mass and inertia values
2. **Joint Limits**: Define appropriate limits to prevent damage in simulation
3. **Collision Meshes**: Use simplified meshes for collision detection to optimize performance
4. **Safety Controllers**: Implement joint position and velocity limits
5. **Gazebo Integration**: Add Gazebo-specific tags if using Gazebo simulator

### Gazebo-Specific Tags

```xml
<gazebo reference="left_wheel">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>
  </plugin>
</gazebo>
```

## Best Practices for URDF Creation

1. **Start Simple**: Begin with a basic model and add complexity gradually
2. **Use Standard Joint Names**: Follow ROS conventions for joint names
3. **Validate Your URDF**: Use tools like `check_urdf` to validate syntax
4. **Use Xacro for Complex Models**: Xacro allows macros and parameters to simplify complex URDFs
5. **Consider Visualization**: Ensure your model looks good in RViz

## Practical Exercises

1. **Create a Simple Robot**: Build a URDF file for a simple wheeled robot with a base and two wheels.

2. **Add Sensors**: Extend your robot model to include a camera or LIDAR sensor.

3. **Build a Planar Arm**: Create a 2-DOF planar arm with proper joints and kinematic chain.

4. **Validate Your Model**: Use the `check_urdf` command to validate your URDF file:
   ```bash
   check_urdf /path/to/your/robot.urdf
   ```

5. **Visualize in RViz**: Load your URDF in RViz to check if it displays correctly.

These exercises will help you understand the structure of URDF files and how to create proper robot descriptions for simulation and real-world use.