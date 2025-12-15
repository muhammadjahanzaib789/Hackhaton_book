# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/004-physical-ai-book/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, contracts/module-contract.md, research.md, quickstart.md

**Tests**: Tests are NOT required for this documentation project. Code examples will be validated during implementation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docs content**: `physical-ai-book/docs/`
- **Code examples**: `physical-ai-book/code-examples/`
- **Static assets**: `physical-ai-book/static/img/`
- **Configuration**: `physical-ai-book/` (root)

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Initialize repository and Docusaurus infrastructure

- [X] T001 Create GitHub repository with README.md, LICENSE (MIT), .gitignore in physical-ai-book/
- [X] T002 Initialize Docusaurus 3.x project with `npx create-docusaurus@latest` in physical-ai-book/
- [X] T003 [P] Configure docusaurus.config.js with project metadata in physical-ai-book/docusaurus.config.js
- [X] T004 [P] Configure sidebars.js for module-based navigation in physical-ai-book/sidebars.js
- [X] T005 [P] Create static/img/ directory structure in physical-ai-book/static/img/
- [X] T006 [P] Create code-examples/ directory structure in physical-ai-book/code-examples/
- [X] T007 Setup GitHub Actions workflow for deployment in physical-ai-book/.github/workflows/deploy.yml
- [X] T008 Verify `npm install && npm start` succeeds locally
- [ ] T009 Verify GitHub Pages deployment pipeline works

**Checkpoint**: Empty Docusaurus site builds and deploys to GitHub Pages

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create book introduction and shared assets that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T010 Create Physical AI introduction in physical-ai-book/docs/intro.md
- [X] T011 [P] Create lesson-template.md guide in physical-ai-book/docs/templates/lesson-template.md
- [X] T012 [P] Create code-example-template.py in physical-ai-book/code-examples/templates/code-example-template.py
- [X] T013 [P] Create shared architecture diagrams in physical-ai-book/static/img/architecture/
- [X] T014 [P] Create humanoid robot URDF base model in physical-ai-book/code-examples/models/humanoid-base.urdf
- [X] T015 Verify intro.md renders correctly and explains core thesis

**Checkpoint**: Foundation ready - intro accessible, templates available, user story implementation can begin

---

## Phase 3: User Story 1 - Learn ROS 2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Reader can create a ROS 2 workspace, write publisher/subscriber, and understand node communication

**Independent Test**: After completing Module 1, reader can create a ROS 2 package with working nodes

### Implementation for User Story 1

- [X] T016 [US1] Create module directory structure in physical-ai-book/docs/module-01-ros2/
- [X] T017 [US1] Create _category_.json for ROS 2 module in physical-ai-book/docs/module-01-ros2/_category_.json
- [X] T018 [P] [US1] Write lesson-01-introduction.md (ROS 2 as nervous system) in physical-ai-book/docs/module-01-ros2/lesson-01-introduction.md
- [X] T019 [P] [US1] Write lesson-02-nodes-topics.md (publisher/subscriber) in physical-ai-book/docs/module-01-ros2/lesson-02-nodes-topics.md
- [X] T020 [P] [US1] Write lesson-03-services-actions.md (service calls, actions) in physical-ai-book/docs/module-01-ros2/lesson-03-services-actions.md
- [X] T021 [P] [US1] Write lesson-04-launch-files.md (multi-node launch) in physical-ai-book/docs/module-01-ros2/lesson-04-launch-files.md
- [X] T022 [P] [US1] Create code-examples/ros2/simple_publisher.py with problem/assumptions/failure documentation
- [X] T023 [P] [US1] Create code-examples/ros2/simple_subscriber.py with problem/assumptions/failure documentation
- [X] T024 [P] [US1] Create code-examples/ros2/service_server.py with problem/assumptions/failure documentation
- [X] T025 [P] [US1] Create code-examples/ros2/action_client.py with problem/assumptions/failure documentation
- [X] T026 [P] [US1] Create code-examples/ros2/humanoid_launch.py (launch file example)
- [ ] T027 [US1] Add failure modes section to each ROS 2 lesson
- [ ] T028 [US1] Add transition sections connecting to Digital Twin module
- [ ] T029 [US1] Verify all ROS 2 lessons follow module-contract.md structure

**Checkpoint**: User Story 1 complete - ROS 2 module fully functional and independently testable

---

## Phase 4: User Story 2 - Simulate Humanoid in Digital Twin (Priority: P2)

**Goal**: Reader can launch a humanoid model in Gazebo, apply joint commands, and observe physics-based responses

**Independent Test**: Humanoid spawns in Gazebo, responds to movement commands with physically accurate motion

### Implementation for User Story 2

- [X] T030 [US2] Create module directory structure in physical-ai-book/docs/module-02-digital-twin/
- [X] T031 [US2] Create _category_.json for Digital Twin module in physical-ai-book/docs/module-02-digital-twin/_category_.json
- [X] T032 [P] [US2] Write lesson-01-gazebo-fundamentals.md in physical-ai-book/docs/module-02-digital-twin/lesson-01-gazebo-fundamentals.md
- [X] T033 [P] [US2] Write lesson-02-urdf-sdf-models.md in physical-ai-book/docs/module-02-digital-twin/lesson-02-urdf-sdf-models.md
- [X] T034 [P] [US2] Write lesson-03-sensor-simulation.md in physical-ai-book/docs/module-02-digital-twin/lesson-03-sensor-simulation.md
- [X] T035 [P] [US2] Write lesson-04-unity-visualization.md in physical-ai-book/docs/module-02-digital-twin/lesson-04-unity-visualization.md
- [ ] T036 [P] [US2] Create code-examples/gazebo/humanoid.urdf with full humanoid model
- [ ] T037 [P] [US2] Create code-examples/gazebo/humanoid_world.sdf with simulation world
- [ ] T038 [P] [US2] Create code-examples/gazebo/spawn_humanoid.py to spawn robot in Gazebo
- [ ] T039 [P] [US2] Create code-examples/gazebo/joint_controller.py for joint commands
- [ ] T040 [P] [US2] Create code-examples/gazebo/sensor_reader.py to read simulated sensors
- [ ] T041 [US2] Add failure modes section to each Digital Twin lesson
- [ ] T042 [US2] Add transition sections connecting to Isaac module
- [ ] T043 [US2] Verify all Digital Twin lessons follow module-contract.md structure

**Checkpoint**: User Story 2 complete - Digital Twin module fully functional

---

## Phase 5: User Story 3 - Integrate NVIDIA Isaac Perception (Priority: P3)

**Goal**: Reader can run Isaac ROS perception nodes and see object detection output from simulated camera feeds

**Independent Test**: Isaac ROS perception pipeline runs on simulated sensor data, Nav2 navigates obstacles

### Implementation for User Story 3

- [X] T044 [US3] Create module directory structure in physical-ai-book/docs/module-03-isaac/
- [X] T045 [US3] Create _category_.json for Isaac module in physical-ai-book/docs/module-03-isaac/_category_.json
- [X] T046 [P] [US3] Write lesson-01-isaac-sim-overview.md in physical-ai-book/docs/module-03-isaac/lesson-01-isaac-sim-overview.md
- [X] T047 [P] [US3] Write lesson-02-isaac-ros-integration.md in physical-ai-book/docs/module-03-isaac/lesson-02-isaac-ros-integration.md
- [X] T048 [P] [US3] Write lesson-03-nav2-navigation.md in physical-ai-book/docs/module-03-isaac/lesson-03-nav2-navigation.md
- [X] T049 [P] [US3] Write lesson-04-synthetic-data.md in physical-ai-book/docs/module-03-isaac/lesson-04-synthetic-data.md
- [ ] T050 [P] [US3] Create code-examples/isaac/perception_pipeline.py for object detection
- [ ] T051 [P] [US3] Create code-examples/isaac/nav2_config.yaml for navigation setup
- [ ] T052 [P] [US3] Create code-examples/isaac/humanoid_navigation.py for Nav2 integration
- [ ] T053 [P] [US3] Create code-examples/isaac/synthetic_data_gen.py for data generation
- [ ] T054 [US3] Add failure modes section to each Isaac lesson
- [ ] T055 [US3] Add transition sections connecting to VLA module
- [ ] T056 [US3] Verify all Isaac lessons follow module-contract.md structure

**Checkpoint**: User Story 3 complete - Isaac module fully functional

---

## Phase 6: User Story 4 - Voice-Control Robot with VLA (Priority: P4)

**Goal**: Reader can speak a command, observe LLM task decomposition, and see robot execute planned actions

**Independent Test**: Voice command → text → LLM decomposition → ROS 2 action execution flow works

### Implementation for User Story 4

- [X] T057 [US4] Create module directory structure in physical-ai-book/docs/module-04-vla/
- [X] T058 [US4] Create _category_.json for VLA module in physical-ai-book/docs/module-04-vla/_category_.json
- [X] T059 [P] [US4] Write lesson-01-voice-to-text.md (Whisper integration) in physical-ai-book/docs/module-04-vla/lesson-01-voice-to-text.md
- [X] T060 [P] [US4] Write lesson-02-llm-task-planning.md in physical-ai-book/docs/module-04-vla/lesson-02-llm-task-planning.md
- [X] T061 [P] [US4] Write lesson-03-ros2-action-bridge.md in physical-ai-book/docs/module-04-vla/lesson-03-ros2-action-bridge.md
- [X] T062 [P] [US4] Write lesson-04-safety-constraints.md in physical-ai-book/docs/module-04-vla/lesson-04-safety-constraints.md
- [ ] T063 [P] [US4] Create code-examples/vla/whisper_transcriber.py for speech-to-text
- [ ] T064 [P] [US4] Create code-examples/vla/llm_planner.py for task decomposition
- [ ] T065 [P] [US4] Create code-examples/vla/action_bridge.py for NL → ROS 2 actions
- [ ] T066 [P] [US4] Create code-examples/vla/safety_guard.py for constraint checking
- [ ] T067 [US4] Add failure modes section to each VLA lesson (LLM unpredictability handling)
- [ ] T068 [US4] Add transition sections connecting to Capstone module
- [ ] T069 [US4] Document deterministic vs probabilistic separation per FR-010
- [ ] T070 [US4] Verify all VLA lessons follow module-contract.md structure

**Checkpoint**: User Story 4 complete - VLA module fully functional

---

## Phase 7: User Story 5 - Complete Capstone Autonomous Humanoid (Priority: P5)

**Goal**: Reader integrates all modules into fully autonomous humanoid responding to voice commands

**Independent Test**: Voice command triggers end-to-end autonomous operation: voice → intent → decomposition → planning → navigation → perception → manipulation

### Implementation for User Story 5

- [X] T071 [US5] Create module directory structure in physical-ai-book/docs/module-05-capstone/
- [X] T072 [US5] Create _category_.json for Capstone module in physical-ai-book/docs/module-05-capstone/_category_.json
- [X] T073 [P] [US5] Write lesson-01-system-integration.md in physical-ai-book/docs/module-05-capstone/lesson-01-system-integration.md
- [X] T074 [P] [US5] Write lesson-02-end-to-end-pipeline.md (7 stages) in physical-ai-book/docs/module-05-capstone/lesson-02-end-to-end-pipeline.md
- [X] T075 [P] [US5] Write lesson-03-failure-modes.md (debugging guide) in physical-ai-book/docs/module-05-capstone/lesson-03-failure-modes.md
- [ ] T076 [P] [US5] Create code-examples/capstone/autonomous_humanoid.py (full integration)
- [ ] T077 [P] [US5] Create code-examples/capstone/pipeline_orchestrator.py (7-stage orchestration)
- [ ] T078 [P] [US5] Create code-examples/capstone/demo_scenario.py ("fetch the cup" demo)
- [ ] T079 [US5] Document all 7 pipeline stages per FR-007 and SC-004
- [ ] T080 [US5] Create architecture diagram showing full system integration in physical-ai-book/static/img/capstone-architecture.png
- [ ] T081 [US5] Verify capstone demonstrates end-to-end autonomy per Constitution VI
- [ ] T082 [US5] Verify all Capstone lessons follow module-contract.md structure

**Checkpoint**: User Story 5 complete - Capstone demonstrates all 7 pipeline stages

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final quality assurance, compliance verification, and release

- [ ] T083 [P] Verify all lessons include 6 required sections per module-contract.md
- [ ] T084 [P] Verify all code examples have problem/assumptions/failure documentation per Constitution V
- [ ] T085 [P] Run markdownlint on all docs/*.md files
- [ ] T086 [P] Verify all diagrams are present and render correctly
- [ ] T087 Create learning outcome traceability matrix in specs/004-physical-ai-book/traceability.md
- [ ] T088 Verify every chapter traces to learning outcome per FR-011
- [ ] T089 Run Quality Bar validation: "Can motivated engineer reproduce without guessing?"
- [ ] T090 Verify spec compliance: all FR-* requirements satisfied
- [ ] T091 Verify success criteria: all SC-* outcomes met
- [ ] T092 Final `npm run build` and production deployment
- [ ] T093 Update README.md with final documentation and usage instructions

**Checkpoint**: Book complete, validated, and deployed to GitHub Pages

---

## Dependencies & Execution Order

### Phase Dependencies

```text
Phase 1: Setup ──────────────────┐
                                 ▼
Phase 2: Foundational ───────────┤ BLOCKS all user stories
                                 ▼
┌────────────────────────────────┴────────────────────────────────┐
│                   User Stories (can run in parallel)            │
│                                                                 │
│  Phase 3: US1 (ROS 2)          → MVP!                          │
│  Phase 4: US2 (Digital Twin)   → Depends on US1 for URDF       │
│  Phase 5: US3 (Isaac)          → Depends on US2 for simulation │
│  Phase 6: US4 (VLA)            → Depends on US3 for perception │
│  Phase 7: US5 (Capstone)       → Depends on ALL previous       │
└────────────────────────────────┬────────────────────────────────┘
                                 ▼
Phase 8: Polish ─────────────────┘
```

### User Story Dependencies

| Story | Can Start After | Dependencies |
|-------|-----------------|--------------|
| US1 (ROS 2) | Phase 2 | Foundation only |
| US2 (Digital Twin) | Phase 2 | Foundation + references US1 URDF |
| US3 (Isaac) | Phase 2 | Foundation + references US2 simulation |
| US4 (VLA) | Phase 2 | Foundation + references US3 perception |
| US5 (Capstone) | US1-US4 complete | All modules must exist |

### Within Each User Story

1. Create directory structure
2. Create _category_.json
3. Write lessons (parallelizable)
4. Create code examples (parallelizable)
5. Add failure modes
6. Add transitions
7. Verify contract compliance

### Parallel Opportunities

**Setup Phase (T001-T009)**:
```bash
# These can run in parallel after T002:
T003, T004, T005, T006, T007
```

**Foundational Phase (T010-T015)**:
```bash
# These can run in parallel after T010:
T011, T012, T013, T014
```

**Each User Story Phase**:
```bash
# After directory/category created, all lessons parallelizable:
# Example for US1:
T018, T019, T020, T021  # Lessons in parallel
T022, T023, T024, T025, T026  # Code examples in parallel
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T009)
2. Complete Phase 2: Foundational (T010-T015)
3. Complete Phase 3: User Story 1 - ROS 2 (T016-T029)
4. **STOP and VALIDATE**: Test US1 independently
5. Deploy MVP to GitHub Pages

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add US1 (ROS 2) → Test independently → Deploy (MVP!)
3. Add US2 (Digital Twin) → Test independently → Deploy
4. Add US3 (Isaac) → Test independently → Deploy
5. Add US4 (VLA) → Test independently → Deploy
6. Add US5 (Capstone) → Test independently → Deploy
7. Polish → Final release

---

## Summary

| Phase | Tasks | Parallel Tasks | Story |
|-------|-------|----------------|-------|
| Setup | T001-T009 | 5 | - |
| Foundational | T010-T015 | 4 | - |
| US1 (ROS 2) | T016-T029 | 9 | P1 |
| US2 (Digital Twin) | T030-T043 | 9 | P2 |
| US3 (Isaac) | T044-T056 | 8 | P3 |
| US4 (VLA) | T057-T070 | 8 | P4 |
| US5 (Capstone) | T071-T082 | 6 | P5 |
| Polish | T083-T093 | 4 | - |
| **Total** | **93** | **53** | - |

### Task Counts by User Story

| User Story | Task Count | MVP Scope |
|------------|------------|-----------|
| US1: ROS 2 Fundamentals | 14 | ✅ MVP |
| US2: Digital Twin | 14 | |
| US3: Isaac Perception | 13 | |
| US4: VLA | 14 | |
| US5: Capstone | 12 | |
| Setup/Foundation/Polish | 26 | ✅ MVP |

### Suggested MVP Scope

Complete these for minimum viable book:
- Phase 1: Setup (9 tasks)
- Phase 2: Foundational (6 tasks)
- Phase 3: US1 - ROS 2 (14 tasks)

**Total MVP Tasks**: 29 of 93 (31%)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- All tasks follow Constitution principles and module-contract.md structure
