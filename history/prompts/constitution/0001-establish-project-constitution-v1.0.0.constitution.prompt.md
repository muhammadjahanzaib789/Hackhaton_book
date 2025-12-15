---
id: 0001
title: Establish Project Constitution v1.0.0
stage: constitution
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 004-physical-ai-book
user: Sheheryar
command: /sp.constitution
labels: ["constitution", "governance", "physical-ai", "robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

/sp.constitution /sp.constitution

Unified Constitution for the Physical AI & Humanoid Robotics Book Project

1. Project Identity

Project Name: Physical AI & Humanoid Robotics
Format: AI-authored technical book
Platform: Docusaurus → GitHub Pages
Authoring System: Claude Code + Spec-Kit Plus
Audience: Advanced AI students, robotics engineers, and researchers transitioning from digital AI to embodied intelligence

Core Thesis:

Intelligence reaches its full potential only when it is embodied. Physical AI bridges the digital brain and the physical body, enabling machines to perceive, reason, and act in the real world.

2. Purpose & Goals

This project exists to:

Teach Physical AI and Embodied Intelligence from first principles to deployment.

Enable students to design, simulate, and control humanoid robots.

Integrate ROS 2, Gazebo, Unity, NVIDIA Isaac, and LLMs into a unified system.

Demonstrate Vision-Language-Action (VLA) pipelines for autonomous humanoids.

Serve as a capstone-quality reference usable in both academia and industry.

Success is defined by:

A complete, deployable book website

Clear conceptual explanations + runnable examples

A coherent capstone narrative culminating in an autonomous humanoid robot

3. Architectural Principles (Non-Negotiable)
3.1 Spec-Driven Development

Every chapter, module, and code example must follow an explicit spec

Specs define:

Learning objectives

Inputs / outputs

Assumptions

Constraints

No undocumented features or unexplained abstractions

3.2 Physical-First AI

Prefer embodied, sensor-driven intelligence over abstract AI

Every AI concept must be grounded in:

Sensors

Actuators

Physics

Environment interaction

3.3 Simulation-to-Real Mindset

Assume all systems will eventually run on real hardware

Emphasize:

Determinism

Latency awareness

Noise, uncertainty, and failure modes

4. Pedagogical Style Rules

Claude Code MUST:

Explain concepts incrementally

Avoid hand-waving or magical thinking

Use diagrams (ASCII or Mermaid where appropriate)

Introduce math only when it adds clarity

Prefer working mental models over formal proofs

Explicitly connect:

Theory → Simulation → Deployment

Tone:

Precise

Engineering-focused

Curious but disciplined

No hype, no sci-fi speculation

5. Content Structure Constitution

The book MUST follow this hierarchy:

Module-Level Rules

Each module includes:

Conceptual Overview

System Architecture

Core Technologies

Hands-on Implementation

Failure Modes & Debugging

Transition to Next Module

Required Modules

The Robotic Nervous System (ROS 2)

The Digital Twin (Gazebo & Unity)

The AI-Robot Brain (NVIDIA Isaac)

Vision-Language-Action (VLA)

Capstone: The Autonomous Humanoid

No module may be skipped or merged.

6. Code & Examples Constitution

All code examples MUST:

Be minimal but complete

Be runnable or clearly marked as pseudocode

Include comments explaining why, not just what

Follow modern best practices

Prefer Python for ROS 2 unless performance demands otherwise

Each major code section must answer:

What problem does this solve?

What assumptions does it make?

What breaks if this fails?

7. AI & LLM Usage Rules

LLMs (GPT, Claude, etc.) are treated as:

Cognitive planners, not magic oracles

Components that:

Interpret intent

Generate plans

Interface with deterministic ROS actions

The book MUST clearly separate:

Deterministic control (ROS, Nav2, controllers)

Probabilistic reasoning (LLMs, perception models)

No anthropomorphism without technical grounding.

8. Capstone Constitution

The final capstone MUST demonstrate:

Voice input → intent understanding

LLM-based task decomposition

ROS 2 action planning

Navigation with obstacle avoidance

Vision-based object recognition

Physical manipulation (simulated)

End-to-end autonomy in simulation

If any link is missing, the capstone is incomplete.

9. Quality Bar

Claude Code must continuously ask:

"Would this explanation allow a motivated engineer to reproduce the system without guessing?"

If the answer is no, the content must be revised.

10. Deployment & Ma

## Response snapshot

Established the Physical AI & Humanoid Robotics Constitution v1.0.0 at `.specify/memory/constitution.md`. The constitution defines:

- **6 Core Principles**: Spec-Driven Development, Physical-First AI, Simulation-to-Real Mindset, Pedagogical Integrity, Code Quality Standards, Capstone Completeness
- **5 Required Modules**: ROS 2 Nervous System, Digital Twin, AI-Robot Brain, VLA, Capstone
- **Project Identity**: AI-authored technical book on Docusaurus for advanced AI/robotics audience
- **Quality Bar**: "Would this explanation allow a motivated engineer to reproduce the system without guessing?"

Sync Impact Report included at top of file showing version 0.0.0 → 1.0.0 (MAJOR - initial ratification).

## Outcome

- ✅ Impact: Established foundational governance document for Physical AI Book project
- 🧪 Tests: N/A (documentation artifact)
- 📁 Files: 1 file created/updated (.specify/memory/constitution.md)
- 🔁 Next prompts: /sp.spec for first module (ROS 2 Nervous System)
- 🧠 Reflection: User-provided constitution was comprehensive; converted placeholder template to concrete principles

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution input was complete and well-structured
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Monitor constitution compliance in subsequent spec/plan commands
