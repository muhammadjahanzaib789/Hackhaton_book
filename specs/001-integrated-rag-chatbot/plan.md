# Implementation Plan: Integrated RAG Chatbot for Published Book

**Branch**: `001-integrated-rag-chatbot` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-integrated-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build an integrated RAG (Retrieval-Augmented Generation) chatbot embedded into the Physical AI & Humanoid Robotics book website. The system will enable readers to ask questions about the entire book content or selected text passages. The backend will use FastAPI with OpenRouter API for LLM responses, Qwen embeddings for semantic search, Qdrant Cloud for vector storage, and Neon Serverless Postgres for metadata and chat logs. The frontend will be a React-based chat widget embeddable into the existing Docusaurus site.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/React 18 (frontend)
**Primary Dependencies**:
- Backend: FastAPI, OpenRouter SDK, Qdrant Client, psycopg3 (Neon Postgres), sentence-transformers or OpenRouter embedding API
- Frontend: React 18, @docusaurus/core 3.9+, axios or fetch for API calls
**Storage**:
- Qdrant Cloud (Free Tier) for vector embeddings
- Neon Serverless Postgres for metadata (chunk mappings, chat logs, source citations)
- Markdown files as source content (existing book structure in `physical-ai-book/docs/`)
**Testing**: pytest (backend), Jest + React Testing Library (frontend)
**Target Platform**:
- Backend: Linux/Docker container (deployable to Vercel, Railway, or AWS Lambda)
- Frontend: Web browser (embedded in Docusaurus static site)
**Project Type**: Web application (backend + frontend)
**Performance Goals**:
- Response time <3s for 95% of queries
- Support 100 concurrent users without degradation beyond 5s
- Embedding generation <500ms per chunk
**Constraints**:
- OpenRouter API rate limits (NEEDS CLARIFICATION - typical limits for free/paid tiers)
- Qdrant Cloud Free Tier limits (NEEDS CLARIFICATION - vectors, collections, memory)
- Neon Serverless Postgres Free Tier limits (NEEDS CLARIFICATION - storage, compute hours)
- Must not expose API keys to client-side code
**Scale/Scope**:
- Initial deployment: ~10,000 queries/month
- Book content: ~100-200 pages, estimated 500-1000 chunks
- Session duration: avg 5-10 queries per user session

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Spec-Driven Development
✅ **PASS** - Feature has explicit spec with learning objectives (user stories), inputs/outputs (query modes, citations), assumptions, and constraints (performance, API limits)

### II. Physical-First AI
⚠️ **CONDITIONAL** - RAG chatbot feature is not directly related to Physical AI embodiment. However, this feature enhances the book's educational infrastructure by enabling better access to Physical AI concepts. Justified as a pedagogical tool rather than a Physical AI system itself.

### III. Simulation-to-Real Mindset
⚠️ **CONDITIONAL** - Not applicable to this feature, as it's a web service for book content, not a robotics system. No sensors, actuators, or hardware interaction.

### IV. Pedagogical Integrity
✅ **PASS** - The chatbot directly supports pedagogical goals by:
- Enabling semantic search across book content
- Providing source citations for verification
- Supporting focused learning via selected-text mode
- Enhancing reader comprehension through Q&A

### V. Code Quality Standards
✅ **PASS** - Implementation will follow code quality standards:
- Minimal, complete examples
- Comments explaining why, not just what
- Clear error handling and failure modes
- Following Python/React best practices

### VI. Capstone Completeness
✅ **N/A** - This feature is infrastructure, not part of the Physical AI capstone project itself.

### Overall Assessment
**Status**: APPROVED with conditional passes

**Justification**: While this feature is not a Physical AI system, it serves as essential infrastructure to make the Physical AI book content more accessible and useful to learners. It aligns with Pedagogical Integrity (Principle IV) and follows Spec-Driven Development (Principle I). The conditional passes on Principles II and III are acceptable because this feature is explicitly educational infrastructure, not core Physical AI content.

---

## Constitution Check - Post-Design Re-evaluation

*Re-checked after Phase 1 design completion*

### I. Spec-Driven Development
✅ **PASS** - Complete data model (data-model.md), API contracts (OpenAPI spec), and quickstart guide generated. All entities have explicit fields, validation rules, and state transitions.

### II. Physical-First AI
⚠️ **CONDITIONAL** - Still not applicable, but the chatbot will enable better understanding of Physical AI concepts by providing instant access to book content. No change from initial assessment.

### III. Simulation-to-Real Mindset
⚠️ **CONDITIONAL** - Still not applicable. No change from initial assessment.

### IV. Pedagogical Integrity
✅ **PASS** - Design demonstrates strong pedagogical support:
- Source citations with chapter/section references enable verification
- Selected-text mode supports focused learning
- Error messages guide users (e.g., "selection too brief")
- Progressive disclosure: simple query interface → detailed citations

### V. Code Quality Standards
✅ **PASS** - Design demonstrates quality standards:
- OpenAPI contract defines clear inputs/outputs
- Data model includes validation rules and constraints
- Failure modes documented (Qdrant down, LLM timeout, etc.)
- Quickstart guide provides runnable examples with expected outputs

### VI. Capstone Completeness
✅ **N/A** - No change from initial assessment.

### Post-Design Summary
All principles satisfied. The design phase has added:
- Complete data model with 5 entities and relationships
- OpenAPI 3.0 contract with 3 endpoints
- Comprehensive quickstart guide with troubleshooting
- Clear storage strategy (Qdrant for vectors, Neon for metadata)

No new concerns identified. Ready to proceed to Phase 2 (tasks generation).

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
│   ├── main.py                    # FastAPI app entry point
│   ├── config.py                  # Environment variables, API keys
│   ├── models/
│   │   ├── chunk.py              # Book content chunk model
│   │   ├── query.py              # Query request/response models
│   │   └── citation.py           # Source citation model
│   ├── services/
│   │   ├── embeddings.py         # Qwen embedding generation via OpenRouter
│   │   ├── retrieval.py          # Qdrant vector search
│   │   ├── llm.py                # OpenRouter LLM API client
│   │   ├── content_indexer.py    # Parse & chunk markdown content
│   │   └── chat_logger.py        # Neon Postgres logging
│   ├── api/
│   │   ├── routes/
│   │   │   ├── health.py         # Health check endpoint
│   │   │   ├── query.py          # POST /api/query
│   │   │   └── index.py          # POST /api/index (admin)
│   │   └── middleware.py         # CORS, rate limiting
│   └── db/
│       ├── neon_client.py        # Postgres connection pool
│       ├── qdrant_client.py      # Qdrant cloud client
│       └── migrations/           # SQL schema migrations
├── tests/
│   ├── unit/                     # Unit tests for services
│   ├── integration/              # API integration tests
│   └── fixtures/                 # Test data
├── requirements.txt
├── Dockerfile
└── .env.example

frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget.tsx        # Main chat widget component
│   │   ├── ChatMessage.tsx       # Individual message display
│   │   ├── ChatInput.tsx         # Input field with submit
│   │   ├── SourceCitation.tsx    # Citation display & links
│   │   └── LoadingIndicator.tsx  # Loading state
│   ├── hooks/
│   │   ├── useChat.ts            # Chat state management
│   │   └── useSelection.ts       # Text selection detection
│   ├── services/
│   │   └── api.ts                # Backend API client
│   ├── types/
│   │   └── chat.ts               # TypeScript interfaces
│   └── styles/
│       └── chat-widget.css       # Widget styles
├── tests/
│   └── components/               # Jest + RTL tests
└── package.json

physical-ai-book/
├── src/
│   └── theme/
│       └── Root.tsx              # Docusaurus theme wrapper (inject widget)
└── (existing Docusaurus structure)
```

**Structure Decision**: Web application structure with separate backend (Python FastAPI) and frontend (React/TypeScript). The frontend will be integrated into the existing Docusaurus site via theme customization. This separation allows independent deployment of the backend API while keeping the frontend embedded in the static site.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations requiring justification. Conditional passes on Physical-First AI and Simulation-to-Real principles are acceptable as this is educational infrastructure, not a Physical AI system.
