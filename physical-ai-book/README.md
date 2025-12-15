# Physical AI & Humanoid Robotics

**Embodied Intelligence in the Real World**

An AI-authored technical book teaching Physical AI and humanoid robotics from first principles to deployment.

## Core Thesis

> Intelligence reaches its full potential only when it is embodied. Physical AI bridges the digital brain and the physical body, enabling machines to perceive, reason, and act in the real world.

## What You'll Learn

This book guides you through designing, simulating, and controlling humanoid robots that integrate:

- **ROS 2** - The robotic nervous system for communication between sensors, actuators, and AI
- **Gazebo & Unity** - Physics-based simulation and visualization (Digital Twin)
- **NVIDIA Isaac** - AI-powered perception and autonomous navigation
- **Vision-Language-Action (VLA)** - Natural language control of robots using LLMs

## Modules

| Module | Title | Description |
|--------|-------|-------------|
| 1 | The Robotic Nervous System | ROS 2 fundamentals: nodes, topics, services, actions |
| 2 | The Digital Twin | Gazebo simulation with humanoid URDF/SDF models |
| 3 | The AI-Robot Brain | NVIDIA Isaac ROS perception and Nav2 navigation |
| 4 | Vision-Language-Action | Voice commands → LLM planning → robot execution |
| 5 | Capstone | Fully autonomous humanoid integrating all modules |

## Prerequisites

- Python programming experience
- Basic AI/ML concepts
- Linux development workflows
- ROS 2 Humble or newer

## Quick Start

```bash
# Install dependencies
npm install

# Start development server
npm start

# Build for production
npm run build
```

## Target Audience

- Advanced AI students
- Robotics engineers
- AI researchers transitioning to Physical AI
- Developers working with ROS 2, Gazebo, Unity, and NVIDIA Isaac

## Capstone Demo

The capstone demonstrates a complete autonomous humanoid pipeline:

```
Voice Input → Intent Understanding → Task Decomposition →
Path Planning → Navigation → Object Detection → Manipulation
```

## Technology Stack

- **Documentation**: Docusaurus 3.x + React 18
- **ROS 2**: Humble (LTS)
- **Simulation**: Gazebo Sim + NVIDIA Isaac Sim
- **Perception**: Isaac ROS + Nav2
- **Voice**: OpenAI Whisper
- **Planning**: LLM-based task decomposition

## Constitution

This book follows the [Physical AI Constitution](/.specify/memory/constitution.md):

1. **Spec-Driven Development** - Every module has explicit learning objectives
2. **Physical-First AI** - All concepts grounded in sensors, actuators, physics
3. **Simulation-to-Real Mindset** - Designed for eventual hardware deployment
4. **Pedagogical Integrity** - Incremental explanations, no hand-waving
5. **Code Quality Standards** - Runnable examples with documented assumptions
6. **Capstone Completeness** - All 7 pipeline stages demonstrated

## License

MIT License - See [LICENSE](./LICENSE) for details.

## Contributing

This book is authored using Claude Code with Spec-Kit Plus governance. Contributions should follow the module contract defined in the specs.

---

**Quality Bar**: Every explanation should allow a motivated engineer to reproduce the system without guessing.
