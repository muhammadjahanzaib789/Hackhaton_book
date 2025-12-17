
# Tasks: Integrated RAG Chatbot for Published Book

**Feature**: 001-integrated-rag-chatbot
**Branch**: `001-integrated-rag-chatbot`
**Input**: Design documents from `/specs/001-integrated-rag-chatbot/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/openapi.yaml, research.md, quickstart.md

**Tests**: Tests are NOT explicitly requested in the spec, so test tasks are omitted. Testing will be manual via quickstart scenarios.

**Organization**: Tasks are grouped by user story (US1-US4) to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- File paths follow plan.md structure: `backend/src/`, `frontend/src/`, `physical-ai-book/src/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backend project structure per plan.md: backend/src/{main.py, config.py, models/, services/, api/routes/, db/}
- [ ] T002 Create frontend project structure per plan.md: frontend/src/{components/, hooks/, services/, types/, styles/}
- [ ] T003 Initialize Python 3.11+ backend with requirements.txt (FastAPI, uvicorn, openai, qdrant-client, psycopg[binary,pool], python-dotenv, tiktoken, pydantic, pytest, httpx)
- [ ] T004 [P] Initialize React 18 + TypeScript frontend with package.json (react, react-dom, typescript, axios, @docusaurus/core, @testing-library/react, jest)
- [ ] T005 [P] Create backend/.env.example with placeholders for OPENROUTER_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, CORS_ORIGINS
- [ ] T006 [P] Create frontend/.env.example with REACT_APP_API_URL placeholder
- [ ] T007 [P] Create backend/Dockerfile for containerized deployment
- [ ] T008 [P] Configure Python linting (flake8/black) and TypeScript linting (ESLint) per Code Quality Standards

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database & External Service Setup

- [ ] T009 Create Neon Postgres database schema in backend/src/db/migrations/001_initial_schema.sql (tables: book_content_chunks, chat_sessions, queries, responses, source_citations with all fields per data-model.md)
- [ ] T010 [P] Implement Neon Postgres connection pool in backend/src/db/neon_client.py (use psycopg3 with connection pooling, error handling)
- [ ] T011 [P] Implement Qdrant Cloud client in backend/src/db/qdrant_client.py (initialize collection "book_chunks" with 768 dimensions, cosine similarity)
- [ ] T012 [P] Create config.py in backend/src/config.py to load environment variables (OpenRouter API key, Qdrant URL/key, DATABASE_URL, CORS origins, chunking params)

### Core Models

- [ ] T013 [P] Create BookContentChunk model in backend/src/models/chunk.py (Pydantic model with fields per data-model.md: chunk_id, text, embedding_vector, chapter_name, section_name, page_number, document_path, chunk_index, token_count, created_at, metadata)
- [ ] T014 [P] Create ChatSession model in backend/src/models/session.py (Pydantic model: session_id, user_id, started_at, last_activity_at, mode, query_count, status with enum validation)
- [ ] T015 [P] Create Query model in backend/src/models/query.py (Pydantic model: query_id, session_id, query_text, selected_text, mode, timestamp, processing_time_ms, error with validation rules per data-model.md)
- [ ] T016 [P] Create Response model in backend/src/models/response.py (Pydantic model: response_id, query_id, response_text, confidence_score, model_used, timestamp, token_count)
- [ ] T017 [P] Create SourceCitation model in backend/src/models/citation.py (Pydantic model: citation_id, response_id, chunk_id, chapter_name, section_name, page_number, relevance_score, excerpt, link)

### API Infrastructure

- [ ] T018 Create FastAPI app in backend/src/main.py (initialize app, include CORS middleware with CORS_ORIGINS from config, register routers, add startup/shutdown events for DB connections)
- [ ] T019 [P] Implement CORS middleware in backend/src/api/middleware.py (allow origins from config, handle preflight requests)
- [ ] T020 [P] Implement health check endpoint GET /v1/health in backend/src/api/routes/health.py (check Qdrant, Neon Postgres, OpenRouter connectivity, return HealthResponse per openapi.yaml)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask Questions About Entire Book Content (Priority: P1) 🎯 MVP

**Goal**: Enable readers to ask questions about the entire book and receive accurate, source-referenced answers in <3s

**Independent Test**:
1. Start backend (uvicorn src.main:app --reload)
2. Run POST /v1/query with query="What is ROS 2?" and session_id (full-book mode)
3. Verify response includes answer text and citations with chapter names
4. Verify processing_time_ms < 3000

### Content Indexing (Admin Task - Required Before Query)

- [ ] T021 [US1] Implement Markdown parser in backend/src/services/content_indexer.py (scan physical-ai-book/docs/ for .md files, extract chapter/section from frontmatter or headers)
- [ ] T022 [US1] Implement recursive text chunker in backend/src/services/content_indexer.py (chunk text into 400 tokens with 80-token overlap per research.md, preserve code blocks intact, extract metadata: chapter, section, document_path, chunk_index)
- [ ] T023 [US1] Implement embedding generation in backend/src/services/embeddings.py (use OpenRouter API with Qwen3 Embedding 8B model via DeepInfra provider, generate 768-dimension vectors, handle rate limits and errors)
- [ ] T024 [US1] Implement chunk storage in backend/src/services/content_indexer.py (store embeddings in Qdrant "book_chunks" collection, store metadata in Neon Postgres book_content_chunks table, handle batch operations)
- [ ] T025 [US1] Create indexing admin endpoint POST /v1/index in backend/src/api/routes/index.py (accept IndexRequest per openapi.yaml, call content_indexer service, return IndexResponse with chunks_indexed count and processing_time_ms)

### Query Processing (Core Functionality)

- [ ] T026 [US1] Implement query embedding generation in backend/src/services/embeddings.py (generate embedding for user query text using same Qwen3 model, handle errors)
- [ ] T027 [US1] Implement vector similarity search in backend/src/services/retrieval.py (query Qdrant with query embedding, retrieve top 10 chunks by cosine similarity, return chunks with relevance scores)
- [ ] T028 [US1] Implement LLM answer generation in backend/src/services/llm.py (use OpenRouter API with GPT-4 or Claude, construct prompt with retrieved chunks + user query, generate answer, handle timeouts and errors per data-model.md failure modes)
- [ ] T029 [US1] Implement chat logging in backend/src/services/chat_logger.py (create/update ChatSession in Neon, insert Query record, insert Response record, insert SourceCitation records with chunk references)
- [ ] T030 [US1] Create query endpoint POST /v1/query in backend/src/api/routes/query.py (accept QueryRequest per openapi.yaml, validate input, call embeddings → retrieval → LLM → chat_logger services, return QueryResponse with citations, track processing_time_ms, handle errors 400/429/500/503)

### Validation & Error Handling

- [ ] T031 [US1] Add input validation to /v1/query endpoint (query text 5-500 chars, session_id is valid UUID, return 400 error per openapi.yaml ErrorResponse if invalid)
- [ ] T032 [US1] Implement "no relevant content found" handling in backend/src/services/llm.py (check if top retrieved chunk has relevance_score < 0.3, return clear message per FR-008 instead of hallucinating)
- [ ] T033 [US1] Add error handling for external service failures in backend/src/api/routes/query.py (Qdrant down → 503 error, OpenRouter down → 503 error, timeout → 500 error, include error messages per openapi.yaml)

**Checkpoint**: User Story 1 (full-book mode) is fully functional and independently testable

---

## Phase 4: User Story 2 - Ask Questions About Selected Text Only (Priority: P2)

**Goal**: Enable readers to highlight text and get clarifications strictly about that selection

**Independent Test**:
1. Run POST /v1/query with query="Explain this in simpler terms", session_id, and selected_text="<paragraph from book>"
2. Verify response answer only references selected text content
3. Verify mode="selected_text" in response

### Selected-Text Mode Implementation

- [ ] T034 [US2] Extend embeddings.py to support selected text embedding (if selected_text provided, generate embedding for selected text instead of querying Qdrant)
- [ ] T035 [US2] Implement in-memory selected text chunk creation in backend/src/services/retrieval.py (when selected_text provided, create temporary chunk with metadata indicating "selected text", skip Qdrant search, use selected text as context)
- [ ] T036 [US2] Update LLM prompt construction in backend/src/services/llm.py (when mode=selected_text, modify prompt to instruct LLM to ONLY use selected text context, add constraint to not reference other book sections)
- [ ] T037 [US2] Add selected_text validation to /v1/query endpoint (if selected_text provided, must be >= 10 chars per data-model.md, return 400 error if too brief with helpful message)
- [ ] T038 [US2] Update query endpoint logic to detect mode (if selected_text in request → mode=selected_text, else mode=full_book, store mode in Query record)

**Checkpoint**: User Story 2 (selected-text mode) is fully functional and independently testable

---

## Phase 5: User Story 3 - View Source Citations and Navigate to Referenced Sections (Priority: P3)

**Goal**: Display citations with chapter/section names and provide clickable links to book locations

**Independent Test**:
1. Run POST /v1/query with any question
2. Verify response citations include chapter_name, section_name (if available), relevance_score
3. Verify citations include link field with Docusaurus route (e.g., "/docs/module-01/lesson-01#section-name")
4. In frontend, click citation link and verify navigation to correct book location

### Citation Enhancement (Backend)

- [ ] T039 [P] [US3] Enhance citation generation in backend/src/services/chat_logger.py (generate link field: map document_path to Docusaurus route format /docs/{module}/{lesson}, extract section anchor from chunk metadata, create full URL with hash anchor)
- [ ] T040 [P] [US3] Add excerpt generation in backend/src/services/chat_logger.py (extract first 500 chars of chunk text as excerpt, store in SourceCitation.excerpt field)
- [ ] T041 [US3] Order citations by relevance_score DESC in QueryResponse (sort citations before returning, highest relevance first per data-model.md validation rules)

### Frontend Citation Display

- [ ] T042 [P] [US3] Create SourceCitation component in frontend/src/components/SourceCitation.tsx (display citation_id, chapter_name, section_name, page_number, relevance_score as confidence indicator, excerpt preview, clickable link to book location)
- [ ] T043 [US3] Implement citation navigation in frontend/src/components/SourceCitation.tsx (onClick handler for link, use React Router or window.location to navigate to Docusaurus page with hash anchor)
- [ ] T044 [US3] Add hover preview for citation excerpt in frontend/src/components/SourceCitation.tsx (tooltip or popover showing full excerpt text on hover, improves UX per User Story 3 acceptance scenario 4)

**Checkpoint**: User Story 3 (source citations with navigation) is fully functional and independently testable

---

## Phase 6: User Story 4 - Fast Response Times for Interactive Learning (Priority: P2)

**Goal**: Optimize system to respond within 2-3s for 95% of queries

**Independent Test**:
1. Run 20 test queries with variety of complexity
2. Measure processing_time_ms for each query using browser dev tools or automated script
3. Verify p95 latency < 3000ms

### Performance Optimization

- [ ] T045 [P] [US4] Add caching for frequently asked queries in backend/src/services/retrieval.py (in-memory LRU cache for query embeddings, cache top 100 queries, 1-hour TTL)
- [ ] T046 [P] [US4] Optimize Qdrant search parameters in backend/src/db/qdrant_client.py (tune ef_search parameter for balance between speed and accuracy, benchmark with test queries)
- [ ] T047 [P] [US4] Implement request timeout enforcement in backend/src/api/routes/query.py (set 5s timeout on LLM requests, return 500 error if exceeded per User Story 4 acceptance scenario 4)
- [ ] T048 [P] [US4] Add loading indicator to frontend in frontend/src/components/LoadingIndicator.tsx (show spinner and estimated time during query processing, manage user expectations per User Story 4 acceptance scenario 3)
- [ ] T049 [US4] Implement connection pooling optimization in backend/src/db/neon_client.py (configure pool size=10, max_overflow=5 for Postgres connections, reduce connection overhead)
- [ ] T050 [US4] Add processing_time_ms tracking to all service methods (measure time for embeddings, retrieval, LLM generation separately, log slow operations >1s for debugging)

**Checkpoint**: User Story 4 (performance requirements) met: <3s for 95% of queries

---

## Phase 7: Frontend Chat Widget (UI for All User Stories)

**Purpose**: Implement React chat widget that integrates all backend functionality

**Dependencies**: Requires US1, US2, US3 backend APIs to be complete

### Core Chat Components

- [ ] T051 [P] Create ChatMessage component in frontend/src/components/ChatMessage.tsx (display user questions and bot responses, render citations using SourceCitation component, style with chat-widget.css)
- [ ] T052 [P] Create ChatInput component in frontend/src/components/ChatInput.tsx (input field for user query, submit button, character count indicator 5-500 chars, loading state during processing)
- [ ] T053 Create ChatWidget main component in frontend/src/components/ChatWidget.tsx (container for chat interface, message history, input area, integrates ChatMessage, ChatInput, LoadingIndicator, SourceCitation components)

### Chat State Management

- [ ] T054 Create useChat hook in frontend/src/hooks/useChat.ts (manage chat state: messages[], session_id generation, query submission via API, response handling, error states)
- [ ] T055 [P] Create useSelection hook in frontend/src/hooks/useSelection.ts (detect text selection on page using window.getSelection(), pass selected text to query if user asks question with text highlighted)
- [ ] T056 Implement backend API client in frontend/src/services/api.ts (axios client for POST /v1/query, POST /v1/index admin, GET /v1/health, handle errors and map to user-friendly messages)

### TypeScript Types & Styling

- [ ] T057 [P] Define TypeScript interfaces in frontend/src/types/chat.ts (QueryRequest, QueryResponse, SourceCitation, ChatMessage, ChatSession types matching openapi.yaml schemas)
- [ ] T058 [P] Style chat widget in frontend/src/styles/chat-widget.css (position as floating widget on page, responsive design, dark/light mode support to match Docusaurus theme, z-index for overlay)

**Checkpoint**: Frontend chat widget complete and functional with all backend features

---

## Phase 8: Docusaurus Integration

**Purpose**: Embed chat widget into existing Physical AI book website

**Dependencies**: Requires frontend chat widget to be fully built

- [ ] T059 Create Docusaurus theme wrapper in physical-ai-book/src/theme/Root.tsx (swizzle Docusaurus Root component, import ChatWidget, inject widget with apiUrl prop from environment)
- [ ] T060 Build frontend for production in frontend/ (npm run build, outputs compiled widget to dist/)
- [ ] T061 Copy built widget assets to physical-ai-book/static/chat-widget/ (copy dist/* files, ensure correct paths for CSS/JS imports)
- [ ] T062 Configure environment variables for Docusaurus in physical-ai-book/ (create .env with REACT_APP_API_URL pointing to deployed backend URL)
- [ ] T063 Test chat widget integration (start Docusaurus dev server, verify widget appears on all pages, test all user stories end-to-end in browser)

**Checkpoint**: Chat widget fully integrated into Docusaurus book website

---

## Phase 9: Deployment & Production Readiness

**Purpose**: Deploy backend and prepare for production use

**Dependencies**: All user stories and integration complete

### Backend Deployment

- [ ] T064 [P] Deploy backend to AWS Lambda for development (create Lambda function with container image, configure API Gateway for /v1/* routes, set environment variables, test with Mangum adapter per research.md)
- [ ] T065 [P] Setup Qdrant Cloud production cluster (create free tier cluster, copy API URL and key, update backend .env)
- [ ] T066 [P] Setup Neon Postgres production database (create free tier project, run migrations, copy connection string, update backend .env)
- [ ] T067 Run initial content indexing in production (call POST /v1/index with content_path="physical-ai-book/docs/", verify ~1000 chunks indexed successfully per quickstart.md expected output)
- [ ] T068 [P] Configure CORS for production frontend domain (update CORS_ORIGINS in backend to include deployed Docusaurus URL, test cross-origin requests)

### Monitoring & Documentation

- [ ] T069 [P] Add logging to all service methods (use Python logging module, log query processing steps, errors, performance metrics, output to stdout for Lambda CloudWatch)
- [ ] T070 [P] Create production deployment guide in specs/001-integrated-rag-chatbot/deployment.md (document Lambda deployment steps, environment variable setup, Qdrant/Neon configuration, indexing procedure)
- [ ] T071 Conduct security audit per FR-015 (verify API keys not exposed to client, check .env files not committed to git, review CORS configuration, test rate limiting if implemented)

**Checkpoint**: System deployed to production and ready for users

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements across all user stories

- [ ] T072 [P] Add rate limiting middleware in backend/src/api/middleware.py (limit to 20 requests/minute per session_id per OpenRouter limits in research.md, return 429 error if exceeded)
- [ ] T073 [P] Implement session cleanup job in backend/src/services/chat_logger.py (mark sessions as "abandoned" if last_activity_at > 30 minutes ago per data-model.md state transitions)
- [ ] T074 [P] Add telemetry for query analytics in backend/src/services/chat_logger.py (log query_text, mode, processing_time_ms, model_used to Neon for basic analytics per Out of Scope clarification)
- [ ] T075 [P] Improve error messages across all endpoints (make errors user-friendly per Code Quality Standards, provide actionable guidance, e.g., "Selection too brief - please select at least 10 characters")
- [ ] T076 [P] Add end-to-end manual testing scenarios based on quickstart.md (test all acceptance scenarios from spec.md, verify success criteria SC-001 through SC-008)
- [ ] T077 Create README.md in backend/ and frontend/ (document setup, running locally, testing, deployment, troubleshooting per quickstart.md structure)

---

## Task Summary

**Total Tasks**: 77
**Tasks by User Story**:
- Setup (Phase 1): 8 tasks
- Foundational (Phase 2): 12 tasks
- US1 - Full-Book Query (Phase 3): 13 tasks
- US2 - Selected-Text Mode (Phase 4): 5 tasks
- US3 - Source Citations & Navigation (Phase 5): 6 tasks
- US4 - Performance Optimization (Phase 6): 6 tasks
- Frontend Chat Widget (Phase 7): 8 tasks
- Docusaurus Integration (Phase 8): 5 tasks
- Deployment (Phase 9): 8 tasks
- Polish & Cross-Cutting (Phase 10): 6 tasks

**Parallel Opportunities**:
- Phase 1: Tasks T004, T005, T006, T007, T008 can run in parallel
- Phase 2: Tasks T010, T011, T012 can run in parallel; T013-T017 (all models) can run in parallel; T019, T020 can run in parallel
- Phase 5: Tasks T039, T040, T042, T043, T044 can run in parallel
- Phase 6: Tasks T045, T046, T047, T048 can run in parallel
- Phase 7: Tasks T051, T052, T055, T057, T058 can run in parallel
- Phase 9: Tasks T064, T065, T066, T068, T069, T070 can run in parallel
- Phase 10: All tasks T072-T077 can run in parallel

---

## Dependencies & Story Completion Order

### Critical Path (Sequential Dependencies)

1. **Phase 1 (Setup)** → MUST complete before all others
2. **Phase 2 (Foundational)** → MUST complete before any user story
3. **Phase 3 (US1 - P1)** → Core functionality, MUST complete first
4. **Phase 4 (US2 - P2)** → Depends on US1 (extends query endpoint)
5. **Phase 5 (US3 - P3)** → Can start after US1 backend complete (independent frontend work)
6. **Phase 6 (US4 - P2)** → Optimization of US1/US2, start after those are functional
7. **Phase 7 (Frontend)** → Depends on US1, US2, US3 backend APIs complete
8. **Phase 8 (Integration)** → Depends on Phase 7 frontend complete
9. **Phase 9 (Deployment)** → Depends on Phase 8 integration complete
10. **Phase 10 (Polish)** → Can run anytime after Phase 3, recommended after Phase 9

### Independent Work Streams (Can Run in Parallel)

- **Stream A**: US1 Backend (Phase 3: T021-T033)
- **Stream B**: After US1 backend complete → US2 Backend (Phase 4: T034-T038) in parallel with US3 Backend (Phase 5: T039-T041)
- **Stream C**: After US1/US2/US3 backend → US4 Performance (Phase 6: T045-T050) in parallel with Frontend (Phase 7: T051-T058)

---

## MVP Scope (Minimum Viable Product)

**Recommended MVP**: User Story 1 ONLY (Phase 1 + Phase 2 + Phase 3)

**MVP Deliverable**:
- Backend API with POST /v1/query (full-book mode only)
- POST /v1/index (admin endpoint for content indexing)
- GET /v1/health
- Command-line testing via curl (no frontend initially)
- Deployed to AWS Lambda
- ~1000 book chunks indexed in Qdrant
- Response time <3s for queries

**MVP Validation**:
```bash
# Test indexing
curl -X POST http://localhost:8000/v1/index \
  -H "Content-Type: application/json" \
  -d '{"content_path": "physical-ai-book/docs/"}'

# Test query
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000"
  }'

# Expected: Response with answer and citations in <3s
```

**Post-MVP Iterations**:
1. **Iteration 2**: Add US2 (selected-text mode) + basic frontend (Phase 4 + Phase 7 minimal)
2. **Iteration 3**: Add US3 (citations with navigation) + full frontend (Phase 5 + Phase 7 complete)
3. **Iteration 4**: Add US4 (performance optimization) + Docusaurus integration (Phase 6 + Phase 8)
4. **Iteration 5**: Production deployment + polish (Phase 9 + Phase 10)

---

## Implementation Strategy

### Incremental Delivery Approach

1. **Week 1**: Setup + Foundational (Phase 1 + Phase 2) - Get infrastructure ready
2. **Week 2**: US1 Backend (Phase 3: T021-T030) - Core RAG functionality
3. **Week 3**: US1 Validation + US2 Backend (Phase 3: T031-T033 + Phase 4) - Extend to selected-text mode
4. **Week 4**: US3 + US4 Backend (Phase 5 + Phase 6) - Citations and performance
5. **Week 5**: Frontend (Phase 7) - Build React chat widget
6. **Week 6**: Integration + Deployment (Phase 8 + Phase 9) - Embed in Docusaurus and deploy
7. **Week 7**: Polish + Testing (Phase 10) - Final improvements and manual testing

### Testing Checkpoints

Since tests are not requested, use these manual testing checkpoints:

**Checkpoint 1 (After Phase 3)**:
- Start backend locally: `uvicorn src.main:app --reload`
- Run POST /v1/index to index book content
- Run POST /v1/query with various questions
- Verify responses include citations and are accurate
- Verify response time <3s

**Checkpoint 2 (After Phase 4)**:
- Test selected-text mode with sample passages
- Verify answers only reference selected text
- Verify mode switching works correctly

**Checkpoint 3 (After Phase 7)**:
- Start frontend: `npm start`
- Test chat widget in isolation
- Verify all user interactions work
- Test text selection detection

**Checkpoint 4 (After Phase 8)**:
- Start Docusaurus site with integrated widget
- Test all user stories end-to-end in browser
- Verify navigation from citations to book pages

**Checkpoint 5 (After Phase 9)**:
- Test deployed backend on AWS Lambda
- Verify production Qdrant and Neon connections
- Test CORS from production Docusaurus domain
- Run acceptance scenarios from spec.md

---

## Edge Cases to Test (From Spec.md)

After implementation is complete, manually test these edge cases:

1. **Long question (>500 words)**: Submit query with >500 chars → verify 400 error with friendly message
2. **Very short selected text (1-2 words)**: Select <10 chars → verify 400 error suggesting more context
3. **Content update**: Re-run POST /v1/index with force_reindex=true → verify chunks updated without disrupting active sessions
4. **Network loss during query**: Disconnect network mid-query → verify clear error message and retry option
5. **Non-English query**: Ask question in non-English → verify polite message requesting English (future enhancement, not blocking)
6. **Multiple concurrent users**: Simulate 10+ concurrent sessions → verify no cross-contamination of context
7. **Service downtime**: Stop Qdrant/Neon/OpenRouter → verify graceful failure messages per data-model.md failure modes
8. **No relevant content**: Ask about topic not in book → verify "not found" message instead of hallucination per FR-008

---

## Success Criteria Verification (From Spec.md)

After full implementation, verify these success criteria:

- [ ] **SC-001**: Run 20 test queries, verify 95% complete in <3s (use processing_time_ms from response)
- [ ] **SC-002**: Test selected-text mode with 5 passages, verify zero cross-contamination from other sections
- [ ] **SC-003**: Manually evaluate 20 answers for accuracy, aim for 85%+ correct using book content
- [ ] **SC-004**: Simulate 100 concurrent users (use load testing tool like Locust), verify response time <5s
- [ ] **SC-005**: Review 10 random responses, verify every response has >=1 citation with correct chapter/section
- [ ] **SC-006**: Audit code and browser dev tools, confirm zero API keys exposed to client (all in backend .env)
- [ ] **SC-007**: Verify chat widget embedded in Docusaurus site without breaking navigation or styling
- [ ] **SC-008**: Check Qdrant dashboard (vectors count), Neon dashboard (storage), OpenRouter usage → confirm within free tier limits

---

## Notes

- **No tests requested**: The spec does not explicitly request TDD or automated tests, so test tasks are omitted. Use manual testing per quickstart.md and checkpoints above.
- **Parallelization**: Many tasks marked [P] can run in parallel to speed up development. Models, frontend components, and documentation can all be developed concurrently.
- **MVP-first**: Focus on US1 (Phase 3) first to deliver core value quickly. US2-US4 are incremental enhancements.
- **File paths**: All file paths are absolute and follow the structure defined in plan.md (backend/, frontend/, physical-ai-book/).
- **Dependencies**: Each phase has clear dependencies. Foundational (Phase 2) blocks all user stories. Frontend (Phase 7) requires backend APIs from US1-US3.
- **Task IDs**: Sequential T001-T077 for easy tracking and reference.
- **Story labels**: [US1], [US2], [US3], [US4] labels enable filtering tasks by user story for independent implementation.
