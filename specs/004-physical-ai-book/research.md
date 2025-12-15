# Research: Physical AI & Humanoid Robotics Book

**Branch**: `004-physical-ai-book` | **Date**: 2025-12-15
**Purpose**: Resolve technical unknowns and establish best practices for implementation

## Technology Decisions

### 1. Documentation Framework

**Decision**: Docusaurus 3.x with React 18

**Rationale**:
- Native Markdown support with MDX for interactive components
- Built-in versioning for book editions
- Excellent code syntax highlighting via Prism
- Static site generation for GitHub Pages deployment
- Active community and Anthropic/Meta backing

**Alternatives Considered**:
| Option | Pros | Cons | Why Rejected |
|--------|------|------|--------------|
| GitBook | Clean UI, easy setup | Commercial limitations, less customizable | Vendor lock-in |
| MkDocs | Python ecosystem | Less React integration | Limited interactivity |
| VitePress | Fast, Vue-based | Smaller ecosystem | React preferred for components |
| Sphinx | Mature, Python docs | Steep learning curve | Overly complex for book format |

### 2. ROS 2 Distribution

**Decision**: ROS 2 Humble Hawksbill (LTS)

**Rationale**:
- Long-term support until May 2027
- Best Isaac ROS compatibility
- Stable Nav2 integration
- Wide hardware support

**Alternatives Considered**:
| Option | Pros | Cons | Why Rejected |
|--------|------|------|--------------|
| ROS 2 Iron | Newer features | Shorter support cycle | Not LTS |
| ROS 2 Jazzy | Latest | Too new, less documentation | Stability concerns |
| ROS 1 Noetic | Large codebase | EOL 2025, no Python 3.10+ | Deprecated |

### 3. Simulation Platform

**Decision**: Gazebo Sim (Ignition) as primary, Isaac Sim for perception

**Rationale**:
- Gazebo Sim: Open source, ROS 2 native, runs on CPU
- Isaac Sim: Required for Isaac ROS perception, synthetic data
- Dual-platform approach matches real-world robotics workflows

**Alternatives Considered**:
| Option | Pros | Cons | Why Rejected |
|--------|------|------|--------------|
| Gazebo Classic | Familiar | Deprecated | End of life |
| Unity only | Great graphics | Complex ROS integration | Not robotics-native |
| Webots | Educational | Limited ROS 2 support | Less industry adoption |

### 4. LLM Integration Strategy

**Decision**: API-based LLM (OpenAI/Anthropic) with local Whisper

**Rationale**:
- API approach avoids GPU requirements for readers
- Whisper can run locally on CPU for speech-to-text
- Clear separation of deterministic (ROS) and probabilistic (LLM) systems
- Easier reproducibility across reader environments

**Alternatives Considered**:
| Option | Pros | Cons | Why Rejected |
|--------|------|------|--------------|
| Local LLaMA | No API costs | GPU required, complex setup | Barrier to entry |
| Azure Cognitive | Enterprise support | Vendor lock-in | Less accessible |
| No LLM | Simpler | Misses VLA requirement | Constitution requirement |

### 5. Humanoid Model

**Decision**: Open-source humanoid URDF (based on simplified biped)

**Rationale**:
- URDF format is ROS 2 standard
- Simplified model reduces simulation complexity
- Can be extended to match commercial robots (Unitree, Boston Dynamics)
- Avoids licensing issues with proprietary models

**Source Options**:
- NAO robot (educational, well-documented)
- Custom simplified biped (18-24 DOF)
- Unitree H1 (if open URDF available)

## Best Practices Research

### Docusaurus for Technical Books

**Key Patterns**:
1. Use `docs/` folder with category.json for navigation
2. Enable code tabs for multi-language examples
3. Use admonitions (:::note, :::warning) for callouts
4. Implement search with Algolia DocSearch
5. Version docs for book editions

**Configuration Essentials**:
```javascript
// docusaurus.config.js key settings
module.exports = {
  presets: [['classic', { docs: { sidebarPath: './sidebars.js' } }]],
  themeConfig: {
    prism: { theme: lightCodeTheme, darkTheme: darkCodeTheme },
    navbar: { title: 'Physical AI Book' }
  }
};
```

### ROS 2 Educational Content

**Key Patterns**:
1. Start with concepts before code
2. Use ASCII diagrams for node graphs
3. Provide complete, minimal examples
4. Include "What can go wrong" sections
5. Show rostopic/ros2 CLI debugging

**Example Structure**:
```text
Lesson: Nodes & Topics
├── Conceptual Overview (mental model)
├── Architecture Diagram (ASCII/Mermaid)
├── Minimal Example (publisher.py)
├── Minimal Example (subscriber.py)
├── Running the Example (commands)
├── What Can Go Wrong (failure modes)
└── Next Steps (transition)
```

### Simulation-to-Real Documentation

**Key Patterns**:
1. Always document sim-real gaps
2. Include physics parameter tuning guides
3. Show sensor noise modeling
4. Provide "real hardware notes" sections
5. Use consistent coordinate frames

### VLA Pipeline Design

**Key Patterns**:
1. Explicit intent parsing before planning
2. Hierarchical task decomposition
3. Safety constraint checking at each level
4. Logging all LLM inputs/outputs
5. Fallback to safe states on uncertainty

**Architecture**:
```text
Voice Input → Whisper → Text
    ↓
Text → LLM Planner → Task Graph
    ↓
Task Graph → Safety Check → Approved Actions
    ↓
Approved Actions → ROS 2 Action Client → Robot
```

## Resolved Clarifications

| Item | Resolution | Source |
|------|------------|--------|
| ROS 2 version | Humble (LTS) | Industry standard |
| Simulation platform | Gazebo Sim + Isaac Sim | Dual-platform coverage |
| Humanoid model | Open-source URDF | Licensing clarity |
| LLM approach | API-based | Reader accessibility |
| Deployment | GitHub Pages | Spec requirement |

## Open Questions for Future Phases

1. **Isaac Sim licensing**: Verify free tier sufficient for book examples
2. **Whisper model size**: Determine smallest model for acceptable accuracy
3. **Manipulation scope**: Define grasp primitives for capstone demo

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Sim Tutorials](https://gazebosim.org/docs)
- [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/)
- [Docusaurus Documentation](https://docusaurus.io/docs)
- [Nav2 Documentation](https://navigation.ros.org/)
