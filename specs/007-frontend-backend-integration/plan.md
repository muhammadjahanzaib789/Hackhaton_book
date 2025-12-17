# Implementation Plan: Frontend and Backend Integration for Embedded RAG Chatbot

**Branch**: `007-frontend-backend-integration` | **Date**: 2025-12-17 | **Spec**: [link to spec](../007-frontend-backend-integration/spec.md)
**Input**: Feature specification from `/specs/007-frontend-backend-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Integration of RAG backend service with Docusaurus frontend to enable embedded chatbot functionality. The system will provide a chat interface within book pages that communicates with the FastAPI backend via REST API, supporting both full-book and selected-text query modes with properly formatted citations.

## Technical Context

**Language/Version**: JavaScript/TypeScript for frontend, Python 3.11 for backend
**Primary Dependencies**: Docusaurus 3.x, React 18, FastAPI 0.104.1, OpenAI SDK
**Storage**: N/A (frontend only stores temporary session state)
**Testing**: Jest for frontend, pytest for backend integration tests
**Target Platform**: Web browsers (Chrome, Firefox, Safari)
**Project Type**: Web (frontend + backend integration)
**Performance Goals**: <3s response time for 95% of requests, <3s interface load time
**Constraints**: Must work in static site environment, REST API communication, no client-side LLM/embedding
**Scale/Scope**: Single book content, multiple concurrent users, responsive design

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the feature requirements and existing architecture:
- Uses existing FastAPI backend service (no new backend required)
- Follows REST API communication pattern
- Integrates with Docusaurus static site generator
- Maintains separation of concerns between frontend and backend
- All requirements from spec are implementable within constraints

## Project Structure

### Documentation (this feature)

```text
specs/007-frontend-backend-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/
├── src/
│   ├── components/
│   │   └── Chatbot/
│   │       ├── ChatbotInterface.jsx
│   │       ├── ChatMessage.jsx
│   │       ├── ChatInput.jsx
│   │       └── CitationRenderer.jsx
│   ├── services/
│   │   ├── api.js
│   │   └── chatbotService.js
│   ├── hooks/
│   │   └── useChatbot.js
│   └── styles/
│       └── chatbot.css
└── docusaurus.config.js

backend/
├── src/
│   ├── api/
│   │   └── agent.py      # Existing RAG agent endpoints
│   ├── models/
│   │   └── agent.py      # Existing agent models
│   └── services/
│       └── agent_service.py # Existing agent service
```

**Structure Decision**: Web application structure with frontend components for Docusaurus integration and backend API endpoints. The frontend will be integrated into the existing Docusaurus site structure, while leveraging existing backend services.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |