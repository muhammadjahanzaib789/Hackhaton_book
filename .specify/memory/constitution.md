<!--
================================================================================
SYNC IMPACT REPORT
================================================================================
Version change: 0.0.0 → 1.0.0 (MAJOR - initial ratification)

Modified principles:
- [PRINCIPLE_1_NAME] → I. Spec-Driven Development
- [PRINCIPLE_2_NAME] → II. Physical-First AI
- [PRINCIPLE_3_NAME] → III. Simulation-to-Real Mindset
- [PRINCIPLE_4_NAME] → IV. Pedagogical Integrity
- [PRINCIPLE_5_NAME] → V. Code Quality Standards
- [PRINCIPLE_6_NAME] → VI. Capstone Completeness

Added sections:
- Project Identity (Core Thesis, Audience, Platform)
- Purpose & Goals (Success Criteria)
- Content Structure Constitution (Module-Level Rules, Required Modules)
- Code & Examples Constitution
- AI & LLM Usage Rules
- Capstone Constitution
- Quality Bar

Removed sections:
- All placeholder sections replaced with concrete content

Templates requiring updates:
- ✅ plan-template.md - Constitution Check section aligns with new principles
- ✅ spec-template.md - User scenarios structure compatible
- ✅ tasks-template.md - Task categorization compatible

Follow-up TODOs: None
================================================================================
-->

# Physical AI & Humanoid Robotics Constitution

## Project Identity

**Project Name**: Physical AI & Humanoid Robotics
**Format**: AI-authored technical book
**Platform**: Docusaurus → GitHub Pages
**Authoring System**: Claude Code + Spec-Kit Plus
**Audience**: Advanced AI students, robotics engineers, and researchers transitioning from digital AI to embodied intelligence

### Core Thesis

Intelligence reaches its full potential only when it is embodied. Physical AI bridges the digital brain and the physical body, enabling machines to perceive, reason, and act in the real world.

## Purpose & Goals

This project exists to:

- Teach Physical AI and Embodied Intelligence from first principles to deployment
- Enable students to design, simulate, and control humanoid robots
- Integrate ROS 2, Gazebo, Unity, NVIDIA Isaac, and LLMs into a unified system
- Demonstrate Vision-Language-Action (VLA) pipelines for autonomous humanoids
- Serve as a capstone-quality reference usable in both academia and industry

### Success Criteria

Success is defined by:

- A complete, deployable book website
- Clear conceptual explanations + runnable examples
- A coherent capstone narrative culminating in an autonomous humanoid robot

## Core Principles

### I. Spec-Driven Development

Every chapter, module, and code example MUST follow an explicit spec.

Specs define:
- Learning objectives
- Inputs / outputs
- Assumptions
- Constraints

**Non-negotiable**: No undocumented features or unexplained abstractions.

### II. Physical-First AI

All AI concepts MUST prefer embodied, sensor-driven intelligence over abstract AI.

Every AI concept MUST be grounded in:
- Sensors
- Actuators
- Physics
- Environment interaction

**Rationale**: Digital-only AI reasoning is insufficient for Physical AI education.

### III. Simulation-to-Real Mindset

All systems MUST be designed with the assumption they will eventually run on real hardware.

Emphasis areas:
- Determinism
- Latency awareness
- Noise, uncertainty, and failure modes

**Rationale**: Simulation-only thinking creates systems that fail on real robots.

### IV. Pedagogical Integrity

Content authoring MUST:
- Explain concepts incrementally
- Avoid hand-waving or magical thinking
- Use diagrams (ASCII or Mermaid where appropriate)
- Introduce math only when it adds clarity
- Prefer working mental models over formal proofs
- Explicitly connect: Theory → Simulation → Deployment

Tone requirements:
- Precise
- Engineering-focused
- Curious but disciplined
- No hype, no sci-fi speculation

### V. Code Quality Standards

All code examples MUST:
- Be minimal but complete
- Be runnable or clearly marked as pseudocode
- Include comments explaining why, not just what
- Follow modern best practices
- Prefer Python for ROS 2 unless performance demands otherwise

Each major code section MUST answer:
- What problem does this solve?
- What assumptions does it make?
- What breaks if this fails?

### VI. Capstone Completeness

The final capstone MUST demonstrate ALL of the following:
- Voice input → intent understanding
- LLM-based task decomposition
- ROS 2 action planning
- Navigation with obstacle avoidance
- Vision-based object recognition
- Physical manipulation (simulated)
- End-to-end autonomy in simulation

**Non-negotiable**: If any link is missing, the capstone is incomplete.

## Content Structure Constitution

### Module-Level Rules

Each module MUST include:
1. Conceptual Overview
2. System Architecture
3. Core Technologies
4. Hands-on Implementation
5. Failure Modes & Debugging
6. Transition to Next Module

### Required Modules

The book MUST contain these modules (no module may be skipped or merged):

1. **The Robotic Nervous System** (ROS 2)
2. **The Digital Twin** (Gazebo & Unity)
3. **The AI-Robot Brain** (NVIDIA Isaac)
4. **Vision-Language-Action** (VLA)
5. **Capstone: The Autonomous Humanoid**

## AI & LLM Usage Rules

LLMs (GPT, Claude, etc.) are treated as:
- Cognitive planners, not magic oracles
- Components that:
  - Interpret intent
  - Generate plans
  - Interface with deterministic ROS actions

The book MUST clearly separate:
- Deterministic control (ROS, Nav2, controllers)
- Probabilistic reasoning (LLMs, perception models)

**Non-negotiable**: No anthropomorphism without technical grounding.

## Quality Bar

Content validation question (MUST be continuously asked):

> "Would this explanation allow a motivated engineer to reproduce the system without guessing?"

If the answer is no, the content MUST be revised.

## Governance

- This Constitution supersedes all other practices for the Physical AI & Humanoid Robotics Book project
- Amendments require:
  - Documentation of the change
  - Version increment following semver rules
  - Migration plan for affected content
- All PRs/reviews MUST verify compliance with these principles
- Complexity MUST be justified against the Spec-Driven Development principle

**Version**: 1.0.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15
