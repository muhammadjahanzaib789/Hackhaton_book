# Implementation Plan: Physical AI & Humanoid Robotics Book

**Branch**: `004-physical-ai-book` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-physical-ai-book/spec.md`

## Summary

Create a spec-driven, AI-authored technical book on Physical AI & Humanoid Robotics using Docusaurus, deployed to GitHub Pages. The book teaches humanoid robot design through 5 modules (ROS 2 → Digital Twin → Isaac → VLA → Capstone) using an incremental vertical slice strategy where each phase produces deployable artifacts.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Docusaurus), Python 3.10+ (ROS 2 examples), C++ (performance-critical ROS nodes)
**Primary Dependencies**: Docusaurus 3.x, React 18, Node.js 18+, ROS 2 Humble, Gazebo, NVIDIA Isaac ROS, OpenAI Whisper
**Storage**: Static Markdown files with YAML frontmatter, Git-based version control
**Testing**: Docusaurus build validation, markdownlint, code example verification scripts
**Target Platform**: GitHub Pages (static hosting), Linux (ROS 2 examples)
**Project Type**: Documentation site (Docusaurus static site generator)
**Performance Goals**: Site loads in <3s, code examples execute in <30s each
**Constraints**: All code must be self-contained, no external runtime dependencies beyond documented prerequisites
**Scale/Scope**: 5 modules, ~20 lessons, ~50 code examples, 1 capstone integration

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Spec-Driven Development | ✅ PASS | Every module has explicit learning objectives in spec.md |
| II. Physical-First AI | ✅ PASS | All content grounded in sensors, actuators, physics per FR-010 |
| III. Simulation-to-Real Mindset | ✅ PASS | Failure modes section required per FR-002, SC-008 |
| IV. Pedagogical Integrity | ✅ PASS | Incremental structure defined, theory→simulation→deployment flow |
| V. Code Quality Standards | ✅ PASS | FR-008, FR-009 mandate runnable, documented code |
| VI. Capstone Completeness | ✅ PASS | All 7 pipeline stages required per FR-007, SC-004 |

**Gate Result**: ✅ ALL PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/004-physical-ai-book/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (content contracts)
│   └── module-contract.md
├── checklists/
│   └── requirements.md  # Specification quality checklist
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
physical-ai-book/
├── docs/                           # Markdown lesson content
│   ├── intro.md                    # Physical AI introduction
│   ├── module-01-ros2/             # The Robotic Nervous System
│   │   ├── _category_.json
│   │   ├── lesson-01-intro.md
│   │   ├── lesson-02-nodes.md
│   │   ├── lesson-03-topics.md
│   │   └── lesson-04-actions.md
│   ├── module-02-digital-twin/     # Gazebo & Unity
│   │   ├── _category_.json
│   │   └── lesson-*.md
│   ├── module-03-isaac/            # NVIDIA Isaac
│   │   ├── _category_.json
│   │   └── lesson-*.md
│   ├── module-04-vla/              # Vision-Language-Action
│   │   ├── _category_.json
│   │   └── lesson-*.md
│   └── module-05-capstone/         # Autonomous Humanoid
│       ├── _category_.json
│       └── lesson-*.md
├── static/
│   └── img/                        # Diagrams and images
├── src/
│   ├── components/                 # Custom React components
│   └── pages/                      # Custom pages
├── code-examples/                  # Runnable code examples
│   ├── ros2/
│   ├── gazebo/
│   ├── isaac/
│   └── vla/
├── docusaurus.config.js
├── sidebars.js
└── package.json
```

**Structure Decision**: Docusaurus documentation site with module-based organization. Each module maps directly to a Constitution-required module. Code examples live in separate directory for independent execution.

## Execution Phases

### Phase 0: Governance & Infrastructure

**Objective**: Ensure spec authority and tooling correctness before content creation.

| Task ID | Task | Acceptance Criteria |
|---------|------|---------------------|
| TASK-001 | Repository Initialization | Git repo with .gitignore, README, LICENSE |
| TASK-002 | Docusaurus Setup | `npm install` succeeds, `npm start` launches dev server |
| TASK-003 | GitHub Pages Deployment | CI/CD pipeline deploys on push to main |

**Exit Criteria**: Live (empty) documentation site accessible via GitHub Pages URL

### Phase 1: Conceptual Foundations

**Objective**: Establish Physical AI mental models and vocabulary.

| Task ID | Task | Acceptance Criteria |
|---------|------|---------------------|
| TASK-010 | Intro – Physical AI Foundations | intro.md explains embodied intelligence, core thesis |

**Validation**: Concepts tied to physics and embodiment, no abstract-only AI explanations

### Phase 2: Control & Middleware (Module 1 - ROS 2)

**Objective**: Build the humanoid robotic nervous system using ROS 2.

| Task ID | Task | Acceptance Criteria |
|---------|------|---------------------|
| TASK-020 | ROS 2 Module Structure | _category_.json, 4 lessons created |
| TASK-021 | Lesson: ROS 2 Introduction | Conceptual overview of ROS 2 as nervous system |
| TASK-022 | Lesson: Nodes & Communication | Publisher/subscriber examples working |
| TASK-023 | Lesson: Services & Actions | Service call and action examples |
| TASK-024 | Lesson: Launch Files | Multi-node launch configuration |
| TASK-025 | Code Examples: ROS 2 | Runnable Python ROS 2 package |

**Validation**: ROS 2 nodes communicate correctly, URDF loads in simulation

### Phase 3: Simulation & Digital Twin (Module 2)

**Objective**: Create a physics-accurate humanoid digital twin.

| Task ID | Task | Acceptance Criteria |
|---------|------|---------------------|
| TASK-030 | Digital Twin Module Structure | _category_.json, lessons created |
| TASK-031 | Lesson: Gazebo Fundamentals | Gazebo concepts, world files |
| TASK-032 | Lesson: URDF/SDF Models | Humanoid model description |
| TASK-033 | Lesson: Sensor Simulation | Camera, IMU, joint sensors |
| TASK-034 | Lesson: Unity Visualization | ROS 2 ↔ Unity bridge |
| TASK-035 | Code Examples: Gazebo | Humanoid spawns and moves |

**Validation**: Physics behaves realistically, sensor data published

### Phase 4: Perception & AI Brain (Module 3)

**Objective**: Enable AI-powered perception and navigation.

| Task ID | Task | Acceptance Criteria |
|---------|------|---------------------|
| TASK-040 | Isaac Module Structure | _category_.json, lessons created |
| TASK-041 | Lesson: Isaac Sim Overview | Isaac Sim environment setup |
| TASK-042 | Lesson: Isaac ROS Integration | Perception pipeline running |
| TASK-043 | Lesson: Nav2 Navigation | Humanoid navigates obstacles |
| TASK-044 | Lesson: Synthetic Data | Data generation pipeline |
| TASK-045 | Code Examples: Isaac | Perception + navigation demo |

**Validation**: Perception models detect objects, Nav2 navigates autonomously

### Phase 5: Language & Planning (Module 4 - VLA)

**Objective**: Enable natural language understanding and planning.

| Task ID | Task | Acceptance Criteria |
|---------|------|---------------------|
| TASK-050 | VLA Module Structure | _category_.json, lessons created |
| TASK-051 | Lesson: Voice-to-Text | Whisper integration |
| TASK-052 | Lesson: LLM Task Planning | Task decomposition pipeline |
| TASK-053 | Lesson: ROS 2 Action Bridge | NL → ROS 2 action flow |
| TASK-054 | Lesson: Safety Constraints | Guardrails documentation |
| TASK-055 | Code Examples: VLA | Voice command executes action |

**Validation**: Natural language → ROS action flow works, plans are explainable

### Phase 6: Capstone Integration (Module 5)

**Objective**: Integrate all systems into a single autonomous humanoid.

| Task ID | Task | Acceptance Criteria |
|---------|------|---------------------|
| TASK-060 | Capstone Module Structure | _category_.json, lessons created |
| TASK-061 | Lesson: System Integration | All modules connected |
| TASK-062 | Lesson: End-to-End Pipeline | 7-stage pipeline documented |
| TASK-063 | Lesson: Failure Modes | Debugging guide |
| TASK-064 | Code Examples: Capstone | Full demo working |

**Validation**: All 7 capstone stages demonstrated, autonomous operation in simulation

### Phase 7: Quality, Compliance & Release

**Objective**: Ensure correctness, traceability, and polish.

| Task ID | Task | Acceptance Criteria |
|---------|------|---------------------|
| TASK-070 | Spec Compliance Review | All FR-* requirements verified |
| TASK-071 | Learning Outcome Traceability | Every chapter traces to learning outcome |
| TASK-072 | Quality Bar Validation | "Motivated engineer" test passes |
| TASK-080 | Final Build & Deployment | Production GitHub Pages release |

**Exit Criteria**: Zero unresolved spec violations, book live on GitHub Pages

## Risk Management

| Risk | Phase | Probability | Impact | Mitigation |
|------|-------|-------------|--------|------------|
| Toolchain instability | Phase 0 | Medium | High | Early CI/CD validation, pin dependency versions |
| Simulation mismatch | Phase 3 | Medium | Medium | Physics tuning + explicit documentation of limitations |
| LLM unpredictability | Phase 5 | High | Medium | Deterministic planners + safety guards + explicit non-determinism warnings |
| Integration complexity | Phase 6 | Medium | High | Incremental integration, interface contracts |
| Code example rot | All | Low | High | Automated verification scripts in CI |

## Claude Code Execution Rules

1. Claude Code executes **one task at a time**
2. No task begins without:
   - Completed dependencies
   - Clear acceptance criteria
3. If ambiguity arises → halt and request clarification
4. Every task output must pass the Quality Bar test

## Definition of Done (Global)

The project is **DONE** when:

- [ ] Book builds successfully using Docusaurus (SC-001)
- [ ] Deployed and accessible on GitHub Pages (SC-001)
- [ ] 100% of modules map to learning outcomes (SC-002)
- [ ] Reader can complete each module independently (SC-003)
- [ ] Capstone demonstrates all 7 pipeline stages (SC-004)
- [ ] All code examples execute without modification (SC-005)
- [ ] Quality Bar test passes for all content (SC-006)
- [ ] Constitution principles verified (SC-007)
- [ ] Failure modes documented for each module (SC-008)

## Complexity Tracking

> No violations detected. All complexity justified by Constitution requirements.

| Aspect | Justification |
|--------|---------------|
| 5 separate modules | Required by Constitution (no merge allowed) |
| Multiple simulation platforms | Gazebo + Isaac required for comprehensive coverage |
| LLM integration | Required for VLA pipeline per Constitution |
