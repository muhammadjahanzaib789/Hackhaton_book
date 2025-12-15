# Data Model: Physical AI & Humanoid Robotics Book

**Branch**: `004-physical-ai-book` | **Date**: 2025-12-15
**Purpose**: Define content entities, relationships, and structure

## Content Entities

### Module

A major section of the book covering a specific Physical AI domain.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | string | Unique identifier | Format: `module-NN` |
| title | string | Display title | Required, 3-50 chars |
| slug | string | URL-safe name | Lowercase, hyphenated |
| order | integer | Sequence in book | 1-5 per Constitution |
| description | string | Brief summary | Required, <200 chars |
| learning_objectives | string[] | What reader will learn | 3-5 objectives |
| prerequisites | string[] | Required prior knowledge | Reference other modules |
| lessons | Lesson[] | Child lessons | 3-5 per module |

**Required Modules** (per Constitution):
1. `module-01`: The Robotic Nervous System (ROS 2)
2. `module-02`: The Digital Twin (Gazebo & Unity)
3. `module-03`: The AI-Robot Brain (NVIDIA Isaac)
4. `module-04`: Vision-Language-Action (VLA)
5. `module-05`: Capstone: The Autonomous Humanoid

---

### Lesson

Individual learning unit within a module with specific learning objectives.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | string | Unique identifier | Format: `lesson-NN` |
| title | string | Display title | Required, 3-50 chars |
| slug | string | URL-safe name | Lowercase, hyphenated |
| order | integer | Sequence in module | 1-N |
| module_id | string | Parent module reference | Required |
| learning_objectives | string[] | Specific goals | 2-4 objectives |
| estimated_time | string | Time to complete | Format: "30 min", "1 hour" |
| sections | Section[] | Content sections | Required |

**Required Sections** (per Constitution Module-Level Rules):
1. Conceptual Overview
2. System Architecture
3. Core Technologies
4. Hands-on Implementation
5. Failure Modes & Debugging
6. Transition to Next Module

---

### Section

A content block within a lesson.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | string | Unique identifier | Auto-generated |
| type | enum | Section type | See types below |
| title | string | Section heading | Required |
| content | string | Markdown content | Required |
| code_examples | CodeExample[] | Associated code | Optional |

**Section Types**:
- `concept`: Theoretical explanation
- `architecture`: System design diagrams
- `implementation`: Hands-on coding
- `debugging`: Failure modes and fixes
- `transition`: Bridge to next topic

---

### CodeExample

Runnable code snippet demonstrating a concept.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | string | Unique identifier | Format: `code-NN` |
| title | string | Example name | Required |
| language | enum | Programming language | python, cpp, yaml, bash |
| filename | string | Source file path | Required |
| description | string | What it demonstrates | Required |
| problem_solved | string | What problem this solves | Required (Constitution) |
| assumptions | string[] | What it assumes | Required (Constitution) |
| failure_modes | string[] | What breaks if this fails | Required (Constitution) |
| input | string | Expected input | Optional |
| output | string | Expected output | Required |
| runnable | boolean | Can be executed | Default: true |

**Code Quality Requirements** (per Constitution V):
- Must be minimal but complete
- Must include comments explaining "why"
- Must follow modern best practices
- Prefer Python for ROS 2

---

### HumanoidRobot

The simulated robotic agent that serves as the subject throughout all modules.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| name | string | Robot identifier | Default: "Atlas-Edu" |
| urdf_path | string | Model file location | Required |
| dof | integer | Degrees of freedom | 18-30 typical |
| sensors | Sensor[] | Attached sensors | Required |
| actuators | Actuator[] | Joint actuators | Required |
| mass | float | Total mass (kg) | Positive |
| height | float | Standing height (m) | Positive |

---

### Sensor

Perception device attached to the humanoid.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| type | enum | Sensor category | camera, imu, lidar, force, joint_state |
| name | string | Sensor identifier | Required |
| topic | string | ROS 2 topic | Required |
| frame_id | string | TF frame | Required |
| rate_hz | float | Publishing rate | Positive |
| noise_model | string | Noise characteristics | Optional |

---

### VLAPipeline

The integrated system connecting voice → language → action for robot control.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| stages | PipelineStage[] | Processing stages | 7 required (Constitution) |
| safety_constraints | string[] | Guard conditions | Required |
| fallback_behavior | string | Safe state on failure | Required |

**Required Stages** (per Constitution VI):
1. Voice input
2. Intent understanding
3. LLM-based task decomposition
4. ROS 2 action planning
5. Navigation with obstacle avoidance
6. Vision-based object recognition
7. Physical manipulation (simulated)

---

## Entity Relationships

```text
Book
 └── Module (1:N)
      └── Lesson (1:N)
           └── Section (1:N)
                └── CodeExample (0:N)

HumanoidRobot
 ├── Sensor (1:N)
 └── Actuator (1:N)

VLAPipeline
 └── PipelineStage (1:7)
```

## Validation Rules

### Module Validation
- All 5 Constitution-required modules must exist
- Each module must have 3-5 lessons
- Learning objectives must be measurable

### Lesson Validation
- All 6 required sections must be present
- Estimated time must be realistic (15min - 2hr)
- Must trace to module learning objectives

### CodeExample Validation
- If `runnable: true`, must execute without errors
- Must document problem_solved, assumptions, failure_modes
- Output must match expected output

### Capstone Validation
- All 7 VLA pipeline stages must be demonstrated
- End-to-end autonomy must be verifiable
- No missing integration links

## State Transitions

### Content Lifecycle
```text
Draft → Review → Approved → Published
  ↑        ↓
  └── Revision ←┘
```

### Module Completion
```text
Incomplete → In Progress → Complete → Validated
                              ↓
                         Integration Ready
```
