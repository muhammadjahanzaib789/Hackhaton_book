/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      link: {
        type: 'generated-index',
        description: 'Learn ROS 2 fundamentals: nodes, topics, services, actions, and launch files for humanoid robot control.',
      },
      items: [
        'module-01-ros2/lesson-01-introduction',
        'module-01-ros2/lesson-02-nodes-topics',
        'module-01-ros2/lesson-03-services-actions',
        'module-01-ros2/lesson-04-launch-files',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      link: {
        type: 'generated-index',
        description: 'Create physics-accurate humanoid simulations using Gazebo and Unity for safe iteration before hardware deployment.',
      },
      items: [
        'module-02-digital-twin/lesson-01-gazebo-fundamentals',
        'module-02-digital-twin/lesson-02-urdf-sdf-models',
        'module-02-digital-twin/lesson-03-sensor-simulation',
        'module-02-digital-twin/lesson-04-unity-visualization',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      link: {
        type: 'generated-index',
        description: 'Integrate NVIDIA Isaac for AI-powered perception and autonomous navigation using Isaac ROS and Nav2.',
      },
      items: [
        'module-03-isaac/lesson-01-isaac-sim-overview',
        'module-03-isaac/lesson-02-isaac-ros-integration',
        'module-03-isaac/lesson-03-nav2-navigation',
        'module-03-isaac/lesson-04-synthetic-data',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {
        type: 'generated-index',
        description: 'Enable natural language control of robots using Whisper, LLM task planning, and ROS 2 action execution.',
      },
      items: [
        'module-04-vla/lesson-01-voice-to-text',
        'module-04-vla/lesson-02-llm-task-planning',
        'module-04-vla/lesson-03-ros2-action-bridge',
        'module-04-vla/lesson-04-safety-constraints',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Capstone',
      link: {
        type: 'generated-index',
        description: 'Integrate all modules into a fully autonomous humanoid that responds to voice commands and completes multi-step tasks.',
      },
      items: [
        'module-05-capstone/lesson-01-system-integration',
        'module-05-capstone/lesson-02-end-to-end-pipeline',
        'module-05-capstone/lesson-03-failure-modes',
      ],
    },
  ],
};

module.exports = sidebars;
