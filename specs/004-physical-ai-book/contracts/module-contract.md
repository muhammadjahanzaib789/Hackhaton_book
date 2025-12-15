# Content Contract: Module Structure

**Version**: 1.0.0 | **Date**: 2025-12-15
**Purpose**: Define the required structure and quality standards for each book module

## Module Contract

Every module in the Physical AI & Humanoid Robotics Book MUST conform to this contract.

### Required Files

```text
docs/module-NN-{slug}/
├── _category_.json          # Navigation metadata
├── lesson-01-{slug}.md      # First lesson
├── lesson-02-{slug}.md      # Second lesson
├── lesson-03-{slug}.md      # Third lesson (minimum)
└── lesson-NN-{slug}.md      # Additional lessons (up to 5)
```

### _category_.json Schema

```json
{
  "label": "Module Title",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Module description (< 200 chars)"
  }
}
```

### Lesson Frontmatter Schema

```yaml
---
sidebar_position: 1
title: "Lesson Title"
description: "Brief description for SEO"
---
```

### Required Lesson Sections

Per Constitution Module-Level Rules, each lesson MUST include:

#### 1. Conceptual Overview
- Mental model for the topic
- ASCII or Mermaid diagrams where appropriate
- Connection to Physical AI thesis

**Contract**:
```markdown
## Conceptual Overview

[Explain the concept incrementally, avoiding hand-waving]

[Include diagram if applicable]

**Key Insight**: [One sentence connecting to embodied intelligence]
```

#### 2. System Architecture
- Component relationships
- Data flow diagrams
- Interface definitions

**Contract**:
```markdown
## System Architecture

```mermaid
graph LR
    A[Component] --> B[Component]
```

**Components**:
- **Component A**: [Purpose]
- **Component B**: [Purpose]
```

#### 3. Core Technologies
- Tools and frameworks used
- Version requirements
- Installation verification

**Contract**:
```markdown
## Core Technologies

| Technology | Version | Purpose |
|------------|---------|---------|
| ROS 2 | Humble | Middleware |

**Verification**:
```bash
ros2 --version
```
```

#### 4. Hands-on Implementation
- Step-by-step instructions
- Complete, runnable code
- Expected output

**Contract**:
```markdown
## Hands-on Implementation

### Step 1: [Action]

[Instructions]

```python
# code-example-id: example-01
# Problem: What problem does this solve?
# Assumptions: What does it assume?
# Failure: What breaks if this fails?

[runnable code]
```

**Expected Output**:
```
[exact output]
```
```

#### 5. Failure Modes & Debugging
- Common errors
- Diagnostic commands
- Resolution steps

**Contract**:
```markdown
## Failure Modes & Debugging

### Issue: [Error Name]

**Symptoms**: [What the reader observes]

**Diagnosis**:
```bash
[diagnostic command]
```

**Resolution**: [How to fix]
```

#### 6. Transition to Next Module
- Summary of learned concepts
- Preview of next topic
- Prerequisites confirmed

**Contract**:
```markdown
## What's Next

**You learned**: [3-5 bullet points]

**Coming up**: [Preview of next lesson/module]

**Prerequisites for next**: [Checklist]
- [ ] Completed this lesson's examples
- [ ] Verified output matches expected
```

## Code Example Contract

Every code example MUST include these elements per Constitution V:

```python
#!/usr/bin/env python3
"""
Code Example: [Title]

Problem Solved: [What problem does this solve?]
Assumptions: [What does it assume?]
Failure Modes: [What breaks if this fails?]

Input: [Expected input, if any]
Output: [Expected output]

Usage:
    [How to run this example]
"""

# [Minimal but complete implementation]
# Comments explain WHY, not just WHAT
```

## Validation Checklist

Before a module is considered complete:

- [ ] All 6 required sections present in each lesson
- [ ] 3-5 lessons per module
- [ ] All code examples are runnable or marked as pseudocode
- [ ] All code examples document problem/assumptions/failure
- [ ] Diagrams present where applicable
- [ ] Failure modes documented
- [ ] Transitions connect to next content
- [ ] Quality Bar test: "Can a motivated engineer reproduce without guessing?"

## Traceability Contract

Per FR-011, every chapter MUST trace back to:

```markdown
---
learning_outcome: "LO-XXX: [Specific, measurable outcome]"
module_goal: "MG-XXX: [Module objective this supports]"
physical_ai_capability: "PAC-XXX: [Real-world robot capability]"
---
```

## Module-Specific Contracts

### Module 1: ROS 2 (The Robotic Nervous System)
- MUST cover: nodes, topics, services, actions, launch files
- MUST include: Python ROS 2 package example
- MUST demonstrate: Node communication working

### Module 2: Digital Twin (Gazebo & Unity)
- MUST include: Humanoid URDF/SDF model
- MUST demonstrate: Physics-accurate simulation
- MUST show: Sensor data publishing

### Module 3: Isaac (AI-Robot Brain)
- MUST include: Isaac ROS perception pipeline
- MUST demonstrate: Nav2 navigation
- MUST show: Object detection on simulated camera

### Module 4: VLA (Vision-Language-Action)
- MUST include: Whisper speech-to-text
- MUST demonstrate: LLM task decomposition
- MUST show: Natural language → ROS 2 action

### Module 5: Capstone
- MUST integrate: All previous modules
- MUST demonstrate: All 7 pipeline stages
- MUST show: End-to-end autonomous operation
