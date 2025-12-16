# Implementation Tasks: Cohere Embedding Pipeline

**Feature**: 005-cohere-embedding-pipeline
**Generated**: 2025-12-17
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

---

## Phase 1: Project Setup

**Purpose**: Initialize backend folder structure with UV package manager

**Dependencies**: None

- [ ] T001 Create backend/main.py skeleton with all 7 function stubs (get_all_urls, extract_text_from_url, chunk_text, embed, create_collection, save_chunk_to_qdrant, main)
- [ ] T002 [P] Update backend/.env.example with Cohere, Qdrant, and TARGET_URL configuration (add COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, TARGET_URL=https://hackhaton-book-wqyh.vercel.app/, CHUNK_SIZE=512, CHUNK_OVERLAP=50)
- [ ] T003 [P] Update backend/pyproject.toml with dependencies (add cohere>=5.0.0, qdrant-client>=1.7.0, beautifulsoup4>=4.12.0, requests>=2.31.0, python-dotenv>=1.0.0, tiktoken>=0.5.0, tqdm>=4.66.0)
- [ ] T004 [P] Create backend/.env from .env.example (copy template, user will add actual API keys)
- [ ] T005 Install dependencies using UV package manager (run: uv pip install -e . in backend/)

**Checkpoint**: Project initialized with all dependencies, main.py skeleton ready for implementation

---

## Phase 2: User Story 1 - URL Crawling and Text Extraction (Priority: P1)

**Goal**: Implement foundation for data ingestion - crawl Docusaurus site and extract clean text

**Independent Test**:
1. Run `python backend/main.py` with TARGET_URL=https://hackhaton-book-wqyh.vercel.app/
2. Verify get_all_urls() returns list of documentation URLs (expected: 50-100 URLs)
3. Verify extract_text_from_url() returns clean text without HTML tags, nav elements
4. Check extracted text preserves code blocks with ``` markers

### URL Discovery Implementation

- [ ] T006 [US1] Implement get_all_urls() sitemap parsing in backend/main.py (fetch sitemap.xml from TARGET_URL, parse XML using xml.etree.ElementTree, extract all <loc> URLs, filter for /docs/ paths, return deduplicated absolute URLs)
- [ ] T007 [US1] Add fallback recursive crawler in get_all_urls() in backend/main.py (if sitemap fails, start from base_url/docs, use requests + BeautifulSoup to find all <a href> links, follow links recursively with visited set, limit depth to 10 levels)
- [ ] T008 [US1] Add retry logic with exponential backoff for URL fetching in get_all_urls() in backend/main.py (wrap requests.get() in try/except, retry 3 times with 1s, 2s, 4s delays on timeout/connection errors)

### Text Extraction Implementation

- [ ] T009 [US1] Implement extract_text_from_url() HTML fetching in backend/main.py (send GET request with User-Agent header "CoherePipelineBot/1.0", handle 404/500 errors, decode as UTF-8 with errors='replace')
- [ ] T010 [US1] Implement HTML parsing and cleaning in extract_text_from_url() in backend/main.py (use BeautifulSoup with lxml parser, find <article> or .markdown element, remove <nav>, <aside>, <footer>, .sidebar, .navbar elements)
- [ ] T011 [US1] Add title extraction in extract_text_from_url() in backend/main.py (find <h1> tag first, fallback to <title> tag, strip whitespace)
- [ ] T012 [US1] Implement code block preservation in extract_text_from_url() in backend/main.py (find all <pre><code> blocks, wrap with ``` markers before extracting text using .get_text(separator="\n\n"))
- [ ] T013 [US1] Add content hash generation in extract_text_from_url() in backend/main.py (compute MD5 hash of extracted text using hashlib.md5(text.encode()).hexdigest(), include in returned dict)

**Checkpoint**: User Story 1 complete - crawler fetches all URLs and extracts clean text with metadata

---

## Phase 3: User Story 2 - Cohere Embedding Generation (Priority: P1)

**Goal**: Transform text into 1024-dimensional vectors using Cohere API

**Independent Test**:
1. Create sample text chunks manually: ["Hello world", "Test embedding", ...]
2. Run embed(chunks) function
3. Verify returned embeddings are List[List[float]] with len(embedding) == 1024
4. Verify batch processing works for 96+ chunks

### Chunking Implementation

- [ ] T014 [US2] Implement chunk_text() tokenization in backend/main.py (initialize tiktoken encoder with cl100k_base, tokenize input text, return token count)
- [ ] T015 [US2] Implement paragraph-based chunking in chunk_text() in backend/main.py (split text by \n\n, iterate paragraphs, accumulate until chunk_size=512 tokens reached, save chunk and start new with 50-token overlap from previous)
- [ ] T016 [US2] Add recursive sentence splitting for large paragraphs in chunk_text() in backend/main.py (if paragraph > 512 tokens, split by (?<=[.!?])\s+ regex, process sentences same as paragraphs)
- [ ] T017 [US2] Add code block detection and preservation in chunk_text() in backend/main.py (detect ```language blocks using regex, keep intact within chunks, do not split code blocks)

### Embedding Generation Implementation

- [ ] T018 [US2] Implement Cohere client initialization in embed() in backend/main.py (create cohere.Client with api_key from os.getenv("COHERE_API_KEY"), handle missing key error)
- [ ] T019 [US2] Implement batch embedding logic in embed() in backend/main.py (split texts into batches of 96, call co.embed(texts=batch, model="embed-english-v3.0", input_type="search_document"), extract response.embeddings)
- [ ] T020 [US2] Add rate limit handling in embed() in backend/main.py (catch CohereAPIError with status_code 429, sleep for Retry-After header value or 5s, retry up to 5 times with exponential backoff)
- [ ] T021 [US2] Add dimension validation in embed() in backend/main.py (assert len(embedding) == 1024 for all returned vectors, log error and skip if validation fails)

**Checkpoint**: User Story 2 complete - text is chunked and converted to 1024-dim Cohere embeddings

---

## Phase 4: User Story 3 - Qdrant Vector Storage (Priority: P1)

**Goal**: Persist embeddings in Qdrant for efficient similarity search

**Independent Test**:
1. Create test collection: create_collection("test_rag_embedding")
2. Generate test embedding: embedding = [0.1] * 1024
3. Save to Qdrant: save_chunk_to_qdrant(chunk_data, embedding, "test_rag_embedding")
4. Query Qdrant to verify vector exists with correct metadata
5. Test similarity search returns relevant results

### Qdrant Collection Setup

- [ ] T022 [US3] Implement Qdrant client initialization in create_collection() in backend/main.py (create QdrantClient with url and api_key from env vars, handle authentication errors)
- [ ] T023 [US3] Implement collection existence check in create_collection() in backend/main.py (call client.get_collections(), check if collection_name exists in list, skip creation if found for idempotency)
- [ ] T024 [US3] Implement collection creation in create_collection() in backend/main.py (call client.create_collection with collection_name="rag_embedding", vectors_config=VectorParams(size=1024, distance=Distance.COSINE))

### Vector Upsert Implementation

- [ ] T025 [US3] Implement point ID generation in save_chunk_to_qdrant() in backend/main.py (generate deterministic ID using hashlib.md5(f"{url}_{chunk_index}".encode()).hexdigest(), convert to integer for Qdrant point_id)
- [ ] T026 [US3] Implement payload construction in save_chunk_to_qdrant() in backend/main.py (create dict with url, title, text, chunk_index, token_count, content_hash, indexed_at timestamp)
- [ ] T027 [US3] Implement Qdrant upsert in save_chunk_to_qdrant() in backend/main.py (create PointStruct with id, vector, payload, call client.upsert with collection_name and points list)
- [ ] T028 [US3] Add retry logic for Qdrant operations in save_chunk_to_qdrant() in backend/main.py (wrap upsert in try/except, retry 3 times with exponential backoff on network errors)

**Checkpoint**: User Story 3 complete - embeddings are stored in Qdrant "rag_embedding" collection with metadata

---

## Phase 5: User Story 4 - End-to-End Pipeline Orchestration (Priority: P2)

**Goal**: Orchestrate complete workflow with progress tracking and error handling

**Independent Test**:
1. Configure .env with all API keys and TARGET_URL
2. Run: python backend/main.py
3. Verify all steps execute in sequence: URLs fetched → text extracted → chunked → embedded → stored
4. Verify progress bar shows real-time status
5. Verify summary statistics at completion (pages crawled, chunks stored, elapsed time)

### Configuration and Validation

- [ ] T029 [US4] Implement environment variable loading in main() in backend/main.py (call load_dotenv(), read COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, TARGET_URL from os.getenv())
- [ ] T030 [US4] Implement configuration validation in main() in backend/main.py (check all required env vars are set and non-empty, if any missing print error message listing missing vars, exit with code 1)

### Pipeline Orchestration

- [ ] T031 [US4] Implement main pipeline loop in main() in backend/main.py (call get_all_urls(TARGET_URL), iterate URLs with tqdm progress bar showing current URL and total count)
- [ ] T032 [US4] Add incremental update logic in main() in backend/main.py (before processing URL, query Qdrant for existing content_hash, if hash matches skip URL and log "Content unchanged", only process URLs with changed/new content)
- [ ] T033 [US4] Implement per-URL processing in main() in backend/main.py (call extract_text_from_url, chunk_text, embed, then loop through chunks calling save_chunk_to_qdrant for each)
- [ ] T034 [US4] Add polite crawling delay in main() in backend/main.py (sleep 1 second between URLs using time.sleep, configurable via CRAWL_DELAY_SECONDS env var)

### Logging and Error Handling

- [ ] T035 [US4] Implement logging configuration in main() in backend/main.py (use logging.basicConfig with INFO level, format with timestamp and level, output to both embedding_pipeline.log file and console)
- [ ] T036 [US4] Add keyboard interrupt handling in main() in backend/main.py (wrap pipeline loop in try/except KeyboardInterrupt, log "Pipeline interrupted by user", print progress summary, exit gracefully with code 0)
- [ ] T037 [US4] Implement summary statistics in main() in backend/main.py (track total_urls, total_chunks, failed_urls counters, compute elapsed_time, log summary at end: "Indexed {total_chunks} chunks from {total_urls} pages in {elapsed_time:.1f}s")
- [ ] T038 [US4] Add error traceback logging in main() in backend/main.py (wrap entire main execution in try/except Exception, log full traceback on unrecoverable errors, exit with code 2)

**Checkpoint**: User Story 4 complete - full pipeline runs with single command, displays progress, handles errors gracefully

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements for production readiness

- [ ] T039 [P] Add docstrings to all 7 functions in backend/main.py (include purpose, args with types, returns, raises, example usage per Python standards)
- [ ] T040 [P] Add type hints to all functions in backend/main.py (use typing module for List, Dict, Any, Optional annotations on params and returns)
- [ ] T041 [P] Create backend/tests/test_main.py with unit tests for each function (test_get_all_urls, test_extract_text_from_url, test_chunk_text, test_embed, test_create_collection, test_save_chunk_to_qdrant using pytest)
- [ ] T042 Add integration test for full pipeline in backend/tests/test_main.py (test_full_pipeline_single_page that runs main() on single test URL, verifies vectors in Qdrant)
- [ ] T043 [P] Create backend/README.md with usage instructions (document setup steps, .env configuration, how to run python main.py, troubleshooting common errors)
- [ ] T044 Add example .env values in backend/.env.example (provide placeholder examples for each variable showing correct format)

---

## Task Summary

**Total Tasks**: 44

**Tasks by User Story**:
- Setup (Phase 1): 5 tasks
- US1 - URL Crawling & Text Extraction (Phase 2): 8 tasks
- US2 - Cohere Embedding Generation (Phase 3): 8 tasks
- US3 - Qdrant Vector Storage (Phase 4): 7 tasks
- US4 - End-to-End Orchestration (Phase 5): 10 tasks
- Polish & Cross-Cutting (Phase 6): 6 tasks

**Parallel Opportunities**:
- Phase 1: Tasks T002, T003 can run in parallel (independent files)
- Phase 6: Tasks T039, T040, T041, T043, T044 can run in parallel (documentation and testing)

---

## Dependencies & Story Completion Order

### Critical Path (Sequential Dependencies)

1. **Phase 1 (Setup)** → MUST complete before all others
2. **Phase 2 (US1 - P1)** → Foundation for text data, MUST complete first
3. **Phase 3 (US2 - P1)** → Depends on Phase 2 (needs chunked text)
4. **Phase 4 (US3 - P1)** → Depends on Phase 3 (needs embeddings)
5. **Phase 5 (US4 - P2)** → Depends on Phases 2, 3, 4 (orchestrates all components)
6. **Phase 6 (Polish)** → Can run anytime after Phase 5, recommended after all core features complete

### MVP Definition

**Minimum Viable Product** = Phase 1 + Phase 2 + Phase 3 + Phase 4 (US1, US2, US3)

This delivers:
- ✅ URL crawling from Docusaurus site
- ✅ Clean text extraction
- ✅ Text chunking (512 tokens, 50 overlap)
- ✅ Cohere embedding generation (1024D)
- ✅ Qdrant vector storage
- ✅ Basic error handling

**What MVP can do**: Index a Docusaurus site and make it searchable via semantic similarity

**What's missing from MVP**: Automated orchestration (manual function calls), progress tracking, incremental updates

### Independent Work Streams (Can Run in Parallel)

- **Stream A**: After Phase 1 → Phase 2 (US1: URL & Text)
- **Stream B**: After Phase 2 → Phase 3 (US2: Embedding) in parallel with Phase 4 setup (US3: Qdrant client)
- **Stream C**: After Phases 2, 3, 4 → Phase 5 (US4: Orchestration)
- **Stream D**: After Phase 5 → Phase 6 (Polish) - all tasks can run in parallel

---

## Implementation Strategy

### Recommended Approach: Incremental Development

**Week 1: MVP Core (Phases 1-4)**
- Days 1-2: Setup + URL Crawling (Phase 1-2, Tasks T001-T013)
- Days 3-4: Embedding Generation (Phase 3, Tasks T014-T021)
- Days 5-6: Qdrant Storage (Phase 4, Tasks T022-T028)
- Day 7: MVP Testing and validation

**Week 2: Orchestration + Polish (Phases 5-6)**
- Days 1-3: Pipeline Orchestration (Phase 5, Tasks T029-T038)
- Days 4-5: Documentation and Testing (Phase 6, Tasks T039-T044)
- Days 6-7: End-to-end testing and deployment prep

### Testing Strategy

Since tests are not explicitly requested in the spec, testing tasks (T041-T042) are provided as optional polish tasks. Developers should:

1. **Unit test each function** as implemented (inline testing during development)
2. **Manual testing** using target URL https://hackhaton-book-wqyh.vercel.app/
3. **Integration testing** by running full pipeline on subset of pages (5-10 pages) before full site
4. **Production validation** by running similarity searches in Qdrant after indexing

### Performance Milestones

After each phase, validate performance targets:

- **After Phase 2**: Single URL text extraction in <1s
- **After Phase 3**: 100 chunks embedded in <2s (batched)
- **After Phase 4**: 100 vectors upserted to Qdrant in <5s
- **After Phase 5**: Full 100-page site indexed in <10 minutes (SC-001)

---

## Success Criteria Mapping

| Success Criterion | Validated By | Phase |
|-------------------|--------------|-------|
| SC-001: Index 100 pages in <10 min | Full pipeline execution time | Phase 5 (T037) |
| SC-002: 95% text extraction accuracy | Manual inspection of extracted text | Phase 2 (T010-T012) |
| SC-003: Handle 10-10K word docs | Test with varying document sizes | Phase 3 (T014-T017) |
| SC-004: Similarity scores >0.7 | Qdrant similarity search validation | Phase 4 (T027) |
| SC-005: Recover from failures in 3 retries | Error injection testing | Phase 2-4 (T008, T020, T028) |
| SC-006: Process 1000 chunks in <15 min | Batch embedding performance | Phase 3 (T019) |
| SC-007: Incremental update 80% faster | Content hash comparison | Phase 5 (T032) |
| SC-008: 100% config validation | Missing env var testing | Phase 5 (T030) |

---

## File Reference

**All implementation in single file**: `backend/main.py`

**Supporting files**:
- `backend/.env` - Configuration (API keys, TARGET_URL)
- `backend/pyproject.toml` - UV dependencies
- `backend/tests/test_main.py` - Optional pytest tests
- `backend/README.md` - Usage documentation
- `backend/embedding_pipeline.log` - Runtime logs

---

**Tasks Status**: ✅ COMPLETE - Ready for implementation with `/sp.implement` or manual coding
