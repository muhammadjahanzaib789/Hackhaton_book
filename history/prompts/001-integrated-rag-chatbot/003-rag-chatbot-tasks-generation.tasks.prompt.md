---
id: 003
title: RAG Chatbot Tasks Generation
stage: tasks
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-integrated-rag-chatbot
branch: 001-integrated-rag-chatbot
user: Sheheryar
command: /sp.tasks
labels: ["tasks", "rag", "chatbot", "user-stories", "implementation-plan"]
links:
  spec: specs/001-integrated-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-integrated-rag-chatbot/tasks.md
tests:
  - User story extraction from spec.md (4 stories: US1-P1, US2-P2, US3-P3, US4-P2)
  - Entity mapping to user stories (5 entities mapped)
  - API contract mapping to user stories (3 endpoints mapped)
  - Task organization validation (77 tasks across 10 phases)
  - Dependency graph verification (critical path identified)
  - Parallel execution opportunities (23 parallelizable tasks marked [P])
---

## Prompt

/sp.tasks ### Phase 1: Book Ingestion
- [ ] Convert book (MD/PDF/HTML) into clean text
- [ ] Chunk text (300–500 tokens, overlap 50–100)
- [ ] Store chunk metadata (chapter, page, section)
- [ ] Generate embeddings using **Qwen embedding model**
- [ ] Upload vectors to **Qdrant Cloud**

### Phase 2: Backend (FastAPI)
- [ ] Setup FastAPI project
- [ ] OpenRouter API integration
- [ ] Qdrant client setup
- [ ] Neon Postgres setup
- [ ] `/chat` endpoint
- [ ] `/embed` endpoint (optional admin)
- [ ] Selected-text-only mode handling

### Phase 3: RAG Logic
- [ ] Query embedding generation
- [ ] Vector similarity search
- [ ] Context filtering (book vs selected text)
- [ ] Prompt template design
- [ ] Answer + citation formatting

### Phase 4: Frontend Integration
- [ ] Chat widget (React / vanilla JS)
- [ ] Text selection detection
- [ ] Send selected text to backend
- [ ] Embedded iframe/component for book site

### Phase 5: Testing & Optimization
- [ ] Accuracy testing
- [ ] Hallucination prevention
- [ ] Token & cost optimization
- [ ] Latency tuning

## Response snapshot

Executed complete task generation workflow for the Integrated RAG Chatbot feature (001-integrated-rag-chatbot). The workflow consisted of:

**Document Loading & Analysis**:
- Loaded all design documents: plan.md, spec.md, data-model.md, research.md, contracts/openapi.yaml
- Analyzed project structure (web app with backend/frontend/physical-ai-book directories)
- Extracted technical stack: Python 3.11+ (FastAPI), React 18 (TypeScript), Qdrant, Neon Postgres, OpenRouter
- Identified 5 core entities: BookContentChunk, ChatSession, Query, Response, SourceCitation
- Identified 3 API endpoints: GET /v1/health, POST /v1/query, POST /v1/index

**User Story Extraction & Prioritization**:
- **US1 (P1)**: Ask Questions About Entire Book Content - Core functionality, MVP
- **US2 (P2)**: Ask Questions About Selected Text Only - Extends US1 with mode switching
- **US3 (P3)**: View Source Citations and Navigate to Referenced Sections - Citation enhancement
- **US4 (P2)**: Fast Response Times for Interactive Learning - Performance optimization

**Entity → User Story Mapping**:
- US1: All 5 entities (core RAG pipeline)
- US2: Query.selected_text field, mode switching logic
- US3: SourceCitation.link, SourceCitation.excerpt fields
- US4: Performance optimizations across all entities

**API Contract → User Story Mapping**:
- US1: POST /v1/query (full_book mode), POST /v1/index, GET /v1/health
- US2: POST /v1/query (selected_text mode)
- US3: SourceCitation schema in QueryResponse
- US4: processing_time_ms tracking, timeout handling

**Task Generation (77 Tasks Across 10 Phases)**:
1. **Phase 1 - Setup** (8 tasks): Project initialization, backend/frontend structure, dependencies, configuration
2. **Phase 2 - Foundational** (12 tasks): Database schema, DB clients, core models, FastAPI infrastructure (BLOCKING - must complete before user stories)
3. **Phase 3 - US1 (P1)** (13 tasks): Content indexing, query processing, LLM generation, validation, error handling - MVP core
4. **Phase 4 - US2 (P2)** (5 tasks): Selected-text mode, in-memory chunk creation, mode detection
5. **Phase 5 - US3 (P3)** (6 tasks): Citation link generation, excerpt creation, frontend citation components
6. **Phase 6 - US4 (P2)** (6 tasks): Query caching, Qdrant optimization, timeout enforcement, loading indicators
7. **Phase 7 - Frontend** (8 tasks): React chat widget, state management hooks, TypeScript types, styling
8. **Phase 8 - Docusaurus Integration** (5 tasks): Theme wrapper, widget embedding, production build
9. **Phase 9 - Deployment** (8 tasks): AWS Lambda deployment, Qdrant/Neon production setup, monitoring, security audit
10. **Phase 10 - Polish** (6 tasks): Rate limiting, session cleanup, error message improvements, documentation

**Dependency Graph & Critical Path**:
- Sequential: Phase 1 → Phase 2 → Phase 3 (US1) → Phase 4 (US2) → Phase 7 (Frontend) → Phase 8 (Integration) → Phase 9 (Deployment)
- US3 backend can run parallel to US2 after US1 complete
- US4 performance can run parallel to US3 and Frontend work
- Phase 10 polish tasks can run anytime after Phase 3

**Parallel Execution Opportunities (23 tasks marked [P])**:
- Phase 1: Tasks T004, T005, T006, T007, T008 (5 parallel)
- Phase 2: Tasks T010, T011, T012 (3 parallel), T013-T017 models (5 parallel), T019, T020 (2 parallel)
- Phase 5: Tasks T039, T040, T042, T043, T044 (5 parallel)
- Phase 6: Tasks T045, T046, T047, T048 (4 parallel)
- Phase 7: Tasks T051, T052, T055, T057, T058 (5 parallel)
- Phase 9: Tasks T064, T065, T066, T068, T069, T070 (6 parallel)
- Phase 10: All tasks T072-T077 (6 parallel)

**MVP Definition**:
- MVP = Phase 1 + Phase 2 + Phase 3 (US1 only)
- Deliverable: Backend API with full-book query mode, content indexing, health check
- Testing: Command-line via curl (no frontend initially)
- Validation: Response time <3s, citations included, accurate answers

**Task Format Compliance**:
- All 77 tasks follow strict checklist format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- Task IDs: Sequential T001-T077
- [P] markers: 23 tasks can run in parallel
- [Story] labels: US1, US2, US3, US4 for user story tasks (setup/foundational/polish have no story label)
- File paths: Absolute paths per plan.md structure (backend/src/, frontend/src/)

**Testing Strategy**:
- No automated tests requested in spec (manual testing only)
- 5 checkpoints defined for manual validation
- 8 edge cases from spec.md to test after implementation
- 8 success criteria (SC-001 through SC-008) to verify at completion

**Implementation Strategy**:
- 7-week incremental delivery plan
- Week 1: Setup + Foundational
- Week 2-4: Backend (US1-US4)
- Week 5: Frontend
- Week 6: Integration + Deployment
- Week 7: Polish + Testing

## Outcome

- ✅ Impact: Generated comprehensive tasks.md with 77 implementation tasks organized by 4 user stories across 10 phases. Clear MVP definition (US1 only), dependency graph, and parallel execution opportunities identified.
- 🧪 Tests: No automated test tasks generated (not requested in spec). 5 manual testing checkpoints defined with specific validation steps.
- 📁 Files: Created tasks.md (77 tasks, 10 phases, strict checklist format)
- 🔁 Next prompts: Begin implementation starting with Phase 1 (Setup) tasks T001-T008. Focus on MVP (Phase 1 + Phase 2 + Phase 3) for initial delivery.
- 🧠 Reflection: Task generation successfully mapped all 4 user stories to specific implementation tasks. User provided manual task outline which was helpful but was replaced with auto-generated tasks based on design documents. All tasks follow strict format with task IDs, parallel markers, story labels, and absolute file paths.

## Evaluation notes (flywheel)

- Failure modes observed: None. User provided manual task outline which was acknowledged and replaced with design-driven task generation per workflow requirements. All design documents loaded successfully.
- Graders run and results (PASS/FAIL): Task format validation - PASS (all 77 tasks follow checklist format). User story mapping - PASS (4 stories mapped to entities/endpoints). Dependency graph - PASS (critical path identified, 23 parallel tasks marked). MVP definition - PASS (Phase 1+2+3 defined as MVP).
- Prompt variant (if applicable): Standard /sp.tasks workflow with user-provided manual task outline context
- Next experiment (smallest change to try): Implement Phase 1 (Setup) tasks T001-T008 to create project structure and validate tech stack setup. Run quickstart.md validation after Phase 2 (Foundational) to verify infrastructure is working before starting US1 implementation.
