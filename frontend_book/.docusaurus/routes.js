import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', 'aba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '173'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '9f2'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'cfb'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '9e0'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '527'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', 'ef5'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'e6e'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '814'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '74d'),
            routes: [
              {
                path: '/docs/digital-twins-unity',
                component: ComponentCreator('/docs/digital-twins-unity', '565'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro-ros2-physical-ai',
                component: ComponentCreator('/docs/intro-ros2-physical-ai', '9a5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-ros-vslam-navigation',
                component: ComponentCreator('/docs/isaac-ros-vslam-navigation', '7f2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/llm-cognitive-planning',
                component: ComponentCreator('/docs/llm-cognitive-planning', '9db'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/nav2-path-planning-humanoid-robots',
                component: ComponentCreator('/docs/nav2-path-planning-humanoid-robots', 'e67'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/nvidia-isaac-sim-photorealistic-simulation',
                component: ComponentCreator('/docs/nvidia-isaac-sim-photorealistic-simulation', '257'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/physics-simulation-gazebo',
                component: ComponentCreator('/docs/physics-simulation-gazebo', 'dbc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/robot-structure-urdf',
                component: ComponentCreator('/docs/robot-structure-urdf', '5b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-communication-model',
                component: ComponentCreator('/docs/ros2-communication-model', '64c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/sensor-simulation-validation',
                component: ComponentCreator('/docs/sensor-simulation-validation', '35b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-capstone-project',
                component: ComponentCreator('/docs/vla-capstone-project', '8a5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/voice-processing-with-whisper',
                component: ComponentCreator('/docs/voice-processing-with-whisper', 'b84'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '9da'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
