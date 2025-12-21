/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'intro-ros2-physical-ai',
        'ros2-communication-model',
        'robot-structure-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'physics-simulation-gazebo',
        'digital-twins-unity',
        'sensor-simulation-validation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'nvidia-isaac-sim-photorealistic-simulation',
        'isaac-ros-vslam-navigation',
        'nav2-path-planning-humanoid-robots',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'voice-processing-with-whisper',
        'llm-cognitive-planning',
        'vla-capstone-project',
      ],
    },
  ],
};

module.exports = sidebars;