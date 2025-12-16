# Implementation Plan: Cohere Embedding Pipeline

**Branch**: `005-cohere-embedding-pipeline` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `specs/005-cohere-embedding-pipeline/spec.md`

## Summary

This feature implements a single-file embedding pipeline (main.py) that crawls a deployed Docusaurus site (https://hackhaton-book-wqyh.vercel.app/), extracts clean text content, generates embeddings using Cohere's embed-english-v3.0 model, and stores vectors in Qdrant for RAG-based retrieval. The pipeline is designed as a standalone indexing script using UV package manager, optimized for simplicity and maintainability through a monolithic architecture.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere==5.x, qdrant-client==1.7+, beautifulsoup4==4.12+, requests==2.31+, python-dotenv==1.0+
**Storage**: Qdrant Cloud (vector database with cosine similarity)
**Testing**: pytest==7.4+ with pytest-asyncio for async operations
**Target Platform**: Backend script (Linux/Windows/macOS) via UV package manager
**Project Type**: Single-file standalone script
**Performance Goals**: 100 pages crawled/indexed in <10 minutes, <1s per page extraction, batch embedding generation
**Constraints**: Cohere API rate limits (100 req/min free tier), Docusaurus server polite crawling (1s delay), single-threaded sequential processing
**Scale/Scope**: ~100-500 pages per Docusaurus site, 512-token chunks with 50-token overlap, 1024-dimensional embeddings

## Constitution Check

**PASS** - Physical AI Book Constitution Principles:

1. **Spec-Driven Development**: Implementation follows explicit spec with clear learning objectives, inputs/outputs, assumptions, and constraints defined in spec.md
2. **Code Quality Standards**: Single-file design is minimal but complete; includes error handling, logging, and retry logic; follows Python best practices
3. **Pedagogical Integrity**: Script demonstrates clear pipeline flow (crawl → extract → embed → store) for educational purposes
4. **No Violations**: Single-file architecture is appropriate for standalone utility script; no unnecessary complexity introduced

## Project Structure

### Documentation (this feature)

```text
specs/005-cohere-embedding-pipeline/
├── spec.md              # Feature specification (COMPLETE)
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0 output (dependency research, API limits)
└── tasks.md             # Phase 2 output (generated via /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── main.py              # SINGLE-FILE IMPLEMENTATION (new file)
├── .env                 # Environment configuration (extended)
├── .env.example         # Environment template (updated)
├── pyproject.toml       # UV package configuration (updated)
├── requirements.txt     # Dependencies (updated)
└── tests/
    └── test_main.py     # Unit and integration tests for main.py (new)
```

**Structure Decision**: Single-file implementation in backend/main.py to keep the embedding pipeline isolated from the existing RAG chatbot codebase (backend/src/). This follows user requirements for a standalone script while leveraging the existing backend/ folder structure and UV package manager setup already configured in the project.

## Complexity Tracking

No complexity violations to justify. Single-file design is the simplest possible implementation per user requirements.

---

## System Architecture

### Pipeline Flow

```text
┌─────────────────┐
│  get_all_urls() │  → Fetch URLs from sitemap.xml or crawl site
└────────┬────────┘
         │
         ▼
┌─────────────────────────┐
│ extract_text_from_url() │  → Clean HTML, remove nav/footer/sidebar
└────────┬────────────────┘
         │
         ▼
┌─────────────────┐
│   chunk_text()  │  → Split into 512-token chunks (50-token overlap)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│     embed()     │  → Generate 1024-dim vectors via Cohere API
└────────┬────────┘
         │
         ▼
┌────────────────────────┐
│ create_collection()    │  → Initialize Qdrant "rag_embedding" collection
└────────┬───────────────┘
         │
         ▼
┌───────────────────────────┐
│ save_chunk_to_qdrant()    │  → Upsert vectors with metadata
└───────────────────────────┘
         │
         ▼
┌─────────────────┐
│     main()      │  → Orchestration + error handling + progress logging
└─────────────────┘
```

### Data Flow

1. **URL Discovery**: GET request to https://hackhaton-book-wqyh.vercel.app/sitemap.xml → parse XML → extract all doc URLs
2. **Text Extraction**: For each URL → requests.get() → BeautifulSoup parse → select main article content → strip HTML tags → extract text
3. **Chunking**: Raw text → tiktoken tokenizer → recursive paragraph splitting → 512-token chunks with 50-token overlap → preserve code blocks
4. **Embedding**: Batch chunks (96 per request) → cohere.embed() → 1024-dim vectors → retry on rate limit (exponential backoff)
5. **Storage**: Qdrant upsert → point_id=hash(url+chunk_index) → vector=embedding → payload={url, title, text, chunk_index, metadata}

### Key Design Decisions

**Decision 1: Single-File Architecture**
- **Rationale**: User explicitly requested single main.py file for simplicity and portability
- **Trade-off**: Less modular but easier to understand, deploy, and maintain for a standalone script
- **Alternative Rejected**: Multi-file with services/ and models/ (adds unnecessary complexity for a utility script)

**Decision 2: Synchronous Execution**
- **Rationale**: Sequential processing simplifies rate limiting, error handling, and progress tracking
- **Trade-off**: Slower than async but simpler implementation and debugging
- **Alternative Rejected**: Async/await with aiohttp (added complexity without significant time savings given 1s crawl delays)

**Decision 3: Sitemap-First URL Discovery**
- **Rationale**: Docusaurus generates sitemap.xml by default; fastest and most reliable way to get all URLs
- **Trade-off**: Falls back to recursive crawling if sitemap not found
- **Alternative Rejected**: Recursive crawling only (slower and may miss dynamically generated pages)

**Decision 4: Cohere Batch Embedding**
- **Rationale**: Cohere supports batch embedding (up to 96 texts per request); significantly reduces API calls and latency
- **Trade-off**: Must handle partial failures within batches
- **Alternative Rejected**: One-by-one embedding (100x more API calls, 10x slower)

**Decision 5: Content-Hash Based Incremental Updates**
- **Rationale**: Hash page content to detect changes; only re-embed modified pages
- **Trade-off**: Requires storing content hashes in Qdrant metadata
- **Alternative Rejected**: Always re-embed everything (wastes API credits and time)

---

## Function Specifications

### 1. `get_all_urls(base_url: str) -> List[str]`

**Purpose**: Fetch all documentation URLs from the target Docusaurus site.

**Algorithm**:
1. Try sitemap.xml first: `GET {base_url}/sitemap.xml`
2. If sitemap exists: parse XML → extract all `<loc>` URLs → filter for /docs/ paths
3. If sitemap fails: fallback to recursive crawling starting from base_url/docs
4. Return deduplicated list of absolute URLs

**Error Handling**:
- HTTP 404/500: Log warning, attempt fallback crawling
- Network timeout: Retry 3 times with exponential backoff (1s, 2s, 4s)
- Invalid XML: Fall back to recursive crawling

**Input**:
- `base_url`: str = "https://hackhaton-book-wqyh.vercel.app/"

**Output**:
- List[str] of absolute URLs (e.g., ["https://hackhaton-book-wqyh.vercel.app/docs/intro", ...])

**Dependencies**: requests, xml.etree.ElementTree, urllib.parse

---

### 2. `extract_text_from_url(url: str) -> Dict[str, Any]`

**Purpose**: Clean HTML and extract main article content from a Docusaurus page.

**Algorithm**:
1. `GET` request to URL with User-Agent header (polite crawling)
2. Parse HTML with BeautifulSoup4
3. Extract title: `<h1>` or `<title>` tag
4. Extract main content: find `article` or `.markdown` selector
5. Remove navigation: exclude `nav`, `aside`, `.sidebar`, `.navbar`
6. Remove footer: exclude `footer`, `.footer`
7. Preserve code blocks: wrap in `<pre><code>` markers
8. Extract plain text using `.get_text()` with separator="\n\n"
9. Compute content hash: `hashlib.md5(text.encode()).hexdigest()`

**Error Handling**:
- HTTP errors: Log and skip URL, continue with next
- Parsing errors: Log warning, return empty text
- Encoding issues: Force UTF-8 decoding with errors='replace'

**Input**:
- `url`: str

**Output**:
- Dict[str, Any] = {
    "url": str,
    "title": str,
    "text": str,
    "content_hash": str (MD5 hex),
    "extracted_at": datetime.now().isoformat()
  }

**Dependencies**: requests, beautifulsoup4, hashlib, datetime

---

### 3. `chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[Dict[str, Any]]`

**Purpose**: Split text into semantically meaningful chunks with overlap for context preservation.

**Algorithm**:
1. Initialize tiktoken encoder: `tiktoken.get_encoding("cl100k_base")` (Cohere uses similar tokenizer)
2. Split by paragraphs: `text.split("\n\n")`
3. Iterate paragraphs:
   - Tokenize paragraph
   - If paragraph > chunk_size: split by sentences recursively
   - If current_chunk + paragraph > chunk_size: save current_chunk, start new chunk with overlap
   - Overlap: include last 50 tokens from previous chunk
4. Preserve code blocks: detect ```language blocks, keep intact within chunks
5. Return chunks with metadata: [{text: str, chunk_index: int, token_count: int}]

**Error Handling**:
- Empty text: Return empty list
- Very large paragraphs (>2048 tokens): Force split by sentences, log warning
- Code blocks spanning multiple chunks: Log warning, split at boundary

**Input**:
- `text`: str (full page text)
- `chunk_size`: int = 512 (Cohere supports 512 tokens per embed call)
- `overlap`: int = 50

**Output**:
- List[Dict[str, Any]] = [
    {"text": str, "chunk_index": int, "token_count": int},
    ...
  ]

**Dependencies**: tiktoken, re

---

### 4. `embed(texts: List[str]) -> List[List[float]]`

**Purpose**: Generate 1024-dimensional embeddings using Cohere embed-english-v3.0 model.

**Algorithm**:
1. Initialize Cohere client: `cohere.Client(api_key=os.getenv("COHERE_API_KEY"))`
2. Batch texts into groups of 96 (Cohere's batch limit)
3. For each batch:
   - Call `co.embed(texts=batch, model="embed-english-v3.0", input_type="search_document")`
   - Extract embeddings: `response.embeddings` (List[List[float]])
   - Retry on rate limit (429): exponential backoff (1s, 2s, 4s, 8s)
4. Flatten batches into single list of embeddings
5. Validate dimensions: assert len(embedding) == 1024

**Error Handling**:
- Rate limit (429): Sleep + retry up to 5 times
- API error (500): Log error, skip batch, continue
- Invalid API key: Raise exception immediately (unrecoverable)
- Network timeout: Retry 3 times with backoff

**Input**:
- `texts`: List[str] (max 96 per batch)

**Output**:
- List[List[float]] (1024 dimensions each)

**Dependencies**: cohere, os, time

---

### 5. `create_collection(collection_name: str = "rag_embedding") -> None`

**Purpose**: Initialize Qdrant collection with proper vector configuration.

**Algorithm**:
1. Initialize Qdrant client: `QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))`
2. Check if collection exists: `client.get_collections()`
3. If exists: log "Collection already exists", skip creation
4. If not exists: create collection:
   ```python
   client.create_collection(
       collection_name=collection_name,
       vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
   )
   ```
5. Log success message

**Error Handling**:
- Collection already exists: Log info, continue (idempotent)
- Authentication error: Raise exception (invalid credentials)
- Network error: Retry 3 times with backoff

**Input**:
- `collection_name`: str = "rag_embedding"

**Output**:
- None (side effect: Qdrant collection created)

**Dependencies**: qdrant_client, os

---

### 6. `save_chunk_to_qdrant(chunk_data: Dict[str, Any], embedding: List[float], collection_name: str = "rag_embedding") -> None`

**Purpose**: Upsert a single chunk's embedding and metadata to Qdrant.

**Algorithm**:
1. Generate point_id: `hash(f"{chunk_data['url']}_{chunk_data['chunk_index']}")` (deterministic for idempotency)
2. Construct payload:
   ```python
   payload = {
       "url": chunk_data["url"],
       "title": chunk_data["title"],
       "text": chunk_data["text"],
       "chunk_index": chunk_data["chunk_index"],
       "token_count": chunk_data["token_count"],
       "content_hash": chunk_data["content_hash"],
       "indexed_at": datetime.now().isoformat()
   }
   ```
3. Upsert to Qdrant:
   ```python
   client.upsert(
       collection_name=collection_name,
       points=[PointStruct(id=point_id, vector=embedding, payload=payload)]
   )
   ```
4. Log success: "Upserted chunk {chunk_index} from {url}"

**Error Handling**:
- Network error: Retry 3 times with exponential backoff
- Invalid vector dimensions: Log error, skip chunk
- Qdrant quota exceeded: Log error, halt pipeline

**Input**:
- `chunk_data`: Dict[str, Any] (from chunk_text + extract_text_from_url)
- `embedding`: List[float] (1024 dimensions)
- `collection_name`: str = "rag_embedding"

**Output**:
- None (side effect: vector stored in Qdrant)

**Dependencies**: qdrant_client, hashlib, datetime

---

### 7. `main() -> None`

**Purpose**: Orchestrate the entire pipeline with progress tracking and error handling.

**Algorithm**:
```python
1. Load environment variables: dotenv.load_dotenv()
2. Validate configuration: check COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, TARGET_URL
3. Initialize clients (Cohere, Qdrant) and create collection
4. Fetch all URLs: urls = get_all_urls(TARGET_URL)
5. For each URL (with tqdm progress bar):
   a. Extract text: doc_data = extract_text_from_url(url)
   b. Check content hash in Qdrant: if hash matches, skip (already indexed)
   c. Chunk text: chunks = chunk_text(doc_data["text"])
   d. Batch embed chunks: embeddings = embed([c["text"] for c in chunks])
   e. For each chunk + embedding: save_chunk_to_qdrant(...)
   f. Sleep 1 second (polite crawling)
6. Log summary: "Indexed {total_chunks} chunks from {total_urls} pages in {elapsed_time}s"
7. Exit gracefully
```

**Error Handling**:
- Missing env vars: Print error, exit with code 1
- Keyboard interrupt (Ctrl+C): Log "Pipeline interrupted", save progress checkpoint, exit gracefully
- Unrecoverable errors: Log full traceback, exit with code 2

**Input**:
- None (reads from environment variables)

**Output**:
- None (side effects: Qdrant populated, logs written)
- Exit code: 0 (success), 1 (config error), 2 (runtime error)

**Dependencies**: All above functions, dotenv, tqdm, logging, sys

---

## Configuration Management

### Environment Variables (.env)

```bash
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here

# Target Docusaurus Site
TARGET_URL=https://hackhaton-book-wqyh.vercel.app/

# Chunking Parameters (optional, defaults provided)
CHUNK_SIZE=512
CHUNK_OVERLAP=50
MIN_CHUNK_SIZE=100

# Rate Limiting (optional)
CRAWL_DELAY_SECONDS=1.0
COHERE_BATCH_SIZE=96
MAX_RETRIES=3
```

### UV Package Manager Setup

**pyproject.toml** (minimal for UV):
```toml
[project]
name = "cohere-embedding-pipeline"
version = "0.1.0"
description = "Docusaurus embedding pipeline using Cohere and Qdrant"
requires-python = ">=3.11"
dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "beautifulsoup4>=4.12.0",
    "requests>=2.31.0",
    "python-dotenv>=1.0.0",
    "tiktoken>=0.5.0",
    "tqdm>=4.66.0"
]

[project.optional-dependencies]
dev = [
    "pytest>=7.4.0",
    "pytest-asyncio>=0.21.0",
    "black>=23.0.0",
    "flake8>=6.0.0"
]
```

**Installation**:
```bash
uv pip install -e .
```

---

## Testing Approach

### Unit Tests (pytest)

**test_main.py**:
```python
def test_get_all_urls_from_sitemap():
    """Test sitemap parsing returns valid URLs."""
    urls = get_all_urls("https://hackhaton-book-wqyh.vercel.app/")
    assert len(urls) > 0
    assert all(url.startswith("https://") for url in urls)

def test_extract_text_from_url():
    """Test HTML cleaning and text extraction."""
    data = extract_text_from_url("https://hackhaton-book-wqyh.vercel.app/docs/intro")
    assert "title" in data
    assert "text" in data
    assert len(data["text"]) > 100  # Meaningful content
    assert "<html>" not in data["text"]  # HTML stripped

def test_chunk_text():
    """Test chunking with overlap."""
    text = "Lorem ipsum " * 1000  # Large text
    chunks = chunk_text(text, chunk_size=400, overlap=50)
    assert len(chunks) > 1
    assert all("token_count" in c for c in chunks)
    assert chunks[0]["chunk_index"] == 0

def test_embed_batch():
    """Test Cohere embedding generation."""
    texts = ["Hello world", "Test embedding"]
    embeddings = embed(texts)
    assert len(embeddings) == 2
    assert all(len(e) == 1024 for e in embeddings)

def test_create_collection_idempotent():
    """Test collection creation is idempotent."""
    create_collection("test_collection")
    create_collection("test_collection")  # Should not raise

def test_save_chunk_to_qdrant():
    """Test vector upsert."""
    chunk_data = {
        "url": "https://example.com/test",
        "title": "Test",
        "text": "Test content",
        "chunk_index": 0,
        "token_count": 10,
        "content_hash": "abc123"
    }
    embedding = [0.1] * 1024
    save_chunk_to_qdrant(chunk_data, embedding, collection_name="test_collection")
    # Verify with Qdrant query
```

---

## Performance Optimization

### Expected Performance

**Baseline** (100-page site):
- URL fetching: 100 × 1s = 100s
- Text extraction: 100 × 0.5s = 50s
- Chunking: 100 × 0.1s = 10s
- Embedding: 1000 chunks / 96 per batch × 1s = 11s
- Qdrant upsert: 1000 × 0.05s = 50s
- **Total**: ~221s (~3.7 minutes)

**Target**: <10 minutes ✅ (spec requirement: SC-001)

---

## Deployment Instructions

### Setup

1. **Clone repository**:
   ```bash
   cd backend
   ```

2. **Install dependencies** (using UV):
   ```bash
   uv pip install -e .
   ```

3. **Configure environment**:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys
   ```

4. **Run pipeline**:
   ```bash
   python main.py
   ```

---

## Next Steps

Ready for `/sp.tasks` to generate implementation task breakdown.

**Plan Status**: ✅ COMPLETE
