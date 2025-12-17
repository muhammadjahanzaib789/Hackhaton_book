# Implementation Plan: OpenAI RAG Agent with FastAPI

**Branch**: `006-openai-rag-agent` | **Date**: 2025-12-17 | **Spec**: [link](./spec.md)
**Input**: Feature specification from `/specs/006-openai-rag-agent/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The OpenAI RAG Agent feature implements a backend RAG system using the OpenAI Agents SDK exposed via FastAPI. The solution integrates with the existing Qdrant-backed retrieval pipeline to answer questions grounded strictly in book content. The implementation will register the retrieval pipeline as an agent tool, create FastAPI endpoints for agent query handling, and ensure responses are properly grounded with citations.

## Technical Context

**Language/Version**: Python 3.11 (based on existing backend infrastructure)
**Primary Dependencies**:
- openai>=1.12.0: For OpenAI Agents SDK functionality
- fastapi>=0.104.1: For creating REST API endpoints
- pydantic>=2.5.2: For data validation and settings management
- qdrant-client>=1.7.0: For interacting with Qdrant vector database
- cohere>=4.24: For retrieval pipeline integration (from Spec-2)

**Storage**: Qdrant vector database (remote/cloud instance) for retrieval, with agent state managed by OpenAI's infrastructure
**Testing**: pytest with integration tests for agent behavior validation
**Target Platform**: Linux server (backend service), but compatible with cross-platform deployment
**Project Type**: backend API service (extension to existing project)
**Performance Goals**:
- 90th percentile query latency < 10 seconds (based on spec SC-005)
- Support 50 concurrent queries without degradation (based on spec SC-006)
**Constraints**:
- Must use OpenAI Agents SDK for agent orchestration (FR-001)
- Integration with existing Qdrant-backed retrieval pipeline (FR-006)
- Responses must be grounded only in retrieved content (FR-003)
- Stateless API design (FR-007)
**Scale/Scope**:
- Moderate query volume (< 1000 queries/hour)
- Book content focused (matching existing retrieval pipeline)
- Single agent instance with tool integration

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution file and research completed, this implementation plan adheres to the core principles:

1. **Library-First**: The agent functionality will be implemented as modular services (rag_agent.py, retrieval_tool.py, agent_service.py) that can be tested independently
2. **CLI Interface**: The backend already implements this principle with FastAPI providing text-based API
3. **Test-First**: Tests will be written for each component following pytest framework, with specific focus on agent behavior validation
4. **Integration Testing**: Focus on contract tests between OpenAI Agents, retrieval pipeline, and FastAPI endpoints as defined in the OpenAPI contract
5. **Observability**: Structured logging will be implemented to ensure debuggability of agent interactions and retrieval processes

All gates pass. The implementation is compatible with existing project architecture and principles. The design leverages the existing Qdrant-backed retrieval pipeline (from Spec-2) as required by the functional requirements.

## Project Structure

### Documentation (this feature)
```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
```text
backend/
├── src/
│   ├── agents/
│   │   ├── __init__.py
│   │   ├── rag_agent.py              # New: OpenAI Agent implementation
│   │   └── retrieval_tool.py         # New: Retrieval tool for agent
│   ├── api/
│   │   ├── __init__.py
│   │   └── agent.py                  # New: FastAPI endpoints for agent
│   ├── models/
│   │   ├── __init__.py
│   │   └── agent.py                  # New: Data models for agent requests/responses
│   ├── services/
│   │   ├── __init__.py
│   │   └── agent_service.py          # New: Agent orchestration service
│   ├── config.py                     # Existing: Configuration management
│   └── main.py                       # Existing: Application entry point
├── tests/
│   ├── unit/
│   ├── integration/
│   │   ├── __init__.py
│   │   └── test_agent.py             # New: Integration tests for agent
│   └── contract/
├── requirements.txt                  # Existing: Dependencies (includes openai-agents)
├── pyproject.toml                    # Existing: Project configuration
└── main.py                           # Existing: Main application
```

**Structure Decision**: The feature will be implemented within the existing backend structure. New modules will be added to handle the agent functionality while reusing existing infrastructure. This approach aligns with the single-project structure already established in the repository and integrates with the existing retrieval pipeline.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |