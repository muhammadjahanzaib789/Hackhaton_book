# Quickstart Guide: RAG Chatbot Development

**Feature**: 001-integrated-rag-chatbot
**Last Updated**: 2025-12-17

This guide walks you through setting up the RAG chatbot for local development.

---

## Prerequisites

### System Requirements
- **OS**: Linux, macOS, or Windows (WSL2 recommended for Windows)
- **Python**: 3.11 or higher
- **Node.js**: 18.0 or higher (for frontend)
- **Git**: Version control

### External Services (Free Tier)
1. **OpenRouter Account**: https://openrouter.ai/
   - Sign up and add $10 credits (unlocks 1,000 req/day)
   - Get API key from dashboard
2. **Qdrant Cloud Account**: https://qdrant.tech/
   - Sign up for free tier (no credit card required)
   - Create a cluster and get API URL + API key
3. **Neon Postgres Account**: https://neon.tech/
   - Sign up for free tier
   - Create a project and get connection string

---

## Step 1: Clone the Repository

```bash
git clone https://github.com/your-username/physical-ai-book.git
cd physical-ai-book
git checkout 001-integrated-rag-chatbot
```

---

## Step 2: Backend Setup

### 2.1 Create Virtual Environment

```bash
cd backend
python -m venv venv

# Activate virtual environment
# On Linux/macOS:
source venv/bin/activate
# On Windows:
venv\Scripts\activate
```

### 2.2 Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

**Expected packages**:
- `fastapi` - Web framework
- `uvicorn[standard]` - ASGI server
- `openai` - OpenRouter SDK (OpenAI-compatible)
- `qdrant-client` - Qdrant vector database client
- `psycopg[binary,pool]` - Neon Postgres client
- `python-dotenv` - Environment variable management
- `tiktoken` - Token counting for OpenAI models
- `pydantic` - Data validation
- `pytest` - Testing framework
- `httpx` - Async HTTP client

### 2.3 Configure Environment Variables

Create a `.env` file in the `backend/` directory:

```bash
cp .env.example .env
```

Edit `.env` with your API keys:

```env
# OpenRouter
OPENROUTER_API_KEY=sk-or-v1-xxxxxxxxxxxxxxxxxxxxxxxxxxxx

# Qdrant Cloud
QDRANT_URL=https://xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx.us-east.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=xxxxxxxxxxxxxxxxxxxxxxxxxxxx

# Neon Postgres
DATABASE_URL=postgresql://user:password@ep-xxxx-xxxx.us-east.aws.neon.tech/dbname?sslmode=require

# Application Settings
ENVIRONMENT=development
LOG_LEVEL=INFO
CHUNK_SIZE=400
CHUNK_OVERLAP=80
MAX_CHUNKS_RETRIEVED=10

# CORS (for local frontend development)
CORS_ORIGINS=http://localhost:3000,http://localhost:3001
```

### 2.4 Initialize Database Schema

Run the database migration to create tables:

```bash
python -m src.db.migrations.run_migrations
```

**Expected output**:
```
✓ Connected to Neon Postgres
✓ Created table: book_content_chunks
✓ Created table: chat_sessions
✓ Created table: queries
✓ Created table: responses
✓ Created table: source_citations
✓ Created indexes
Database schema initialized successfully
```

### 2.5 Initialize Qdrant Collection

Create the vector collection:

```bash
python -m src.db.qdrant_client init
```

**Expected output**:
```
✓ Connected to Qdrant Cloud
✓ Created collection: book_chunks
  - Vector size: 768
  - Distance: Cosine
  - HNSW config: M=16, ef_construct=100
Collection initialized successfully
```

### 2.6 Index Book Content

Parse and index the book content:

```bash
python -m src.services.content_indexer --content-path ../physical-ai-book/docs/
```

**Expected output**:
```
Scanning content directory: ../physical-ai-book/docs/
Found 47 Markdown files

Processing files:
[████████████████████████████████████████] 47/47 (100%)

✓ Parsed 47 files
✓ Generated 1,247 chunks
✓ Created embeddings (Qwen3 Embedding 8B)
✓ Stored in Qdrant: 1,247 vectors
✓ Stored in Neon Postgres: 1,247 metadata records

Indexing completed in 45.3 seconds
```

### 2.7 Start the Backend Server

```bash
uvicorn src.main:app --reload --port 8000
```

**Expected output**:
```
INFO:     Will watch for changes in these directories: ['/path/to/backend']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 2.8 Test the Backend

Open http://localhost:8000/docs in your browser to see the Swagger UI.

**Test the health endpoint**:
```bash
curl http://localhost:8000/v1/health
```

**Expected response**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-17T12:34:56Z",
  "dependencies": {
    "qdrant": {"status": "up", "latency_ms": 45},
    "neon_postgres": {"status": "up", "latency_ms": 23},
    "openrouter": {"status": "up", "latency_ms": 120}
  }
}
```

**Test a query**:
```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000"
  }'
```

**Expected response**:
```json
{
  "query_id": "660e8400-e29b-41d4-a716-446655440001",
  "response_text": "ROS 2 (Robot Operating System 2) is an open-source framework for building robot applications...",
  "citations": [
    {
      "chapter_name": "Chapter 1: Introduction to ROS 2",
      "relevance_score": 0.94,
      "link": "/docs/module-01/lesson-01"
    }
  ],
  "mode": "full_book",
  "processing_time_ms": 2340
}
```

---

## Step 3: Frontend Setup

### 3.1 Install Frontend Dependencies

```bash
cd ../frontend
npm install
```

**Expected packages**:
- `react` - UI framework
- `react-dom` - React rendering
- `typescript` - Type safety
- `axios` - HTTP client
- `@docusaurus/core` - Docusaurus integration
- `@testing-library/react` - Testing utilities

### 3.2 Configure Frontend Environment

Create a `.env.local` file in the `frontend/` directory:

```bash
cp .env.example .env.local
```

Edit `.env.local`:

```env
REACT_APP_API_URL=http://localhost:8000/v1
```

### 3.3 Start the Frontend Development Server

```bash
npm start
```

**Expected output**:
```
Compiled successfully!

You can now view the chat widget in the browser.

  Local:            http://localhost:3001
  On Your Network:  http://192.168.1.x:3001
```

Open http://localhost:3001 to see the chat widget in isolation.

---

## Step 4: Integrate Chat Widget into Docusaurus Site

### 4.1 Build the Chat Widget

```bash
cd frontend
npm run build
```

This creates a `dist/` folder with the compiled widget.

### 4.2 Copy Widget to Docusaurus

```bash
cp -r dist/* ../physical-ai-book/static/chat-widget/
```

### 4.3 Inject Widget into Docusaurus Theme

Edit `physical-ai-book/src/theme/Root.tsx`:

```tsx
import React from 'react';
import ChatWidget from '@site/static/chat-widget/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget apiUrl={process.env.REACT_APP_API_URL || 'http://localhost:8000/v1'} />
    </>
  );
}
```

### 4.4 Start Docusaurus

```bash
cd ../physical-ai-book
npm start
```

Open http://localhost:3000 to see the book website with the integrated chat widget.

---

## Step 5: Run Tests

### Backend Tests

```bash
cd backend
pytest tests/ -v
```

**Expected output**:
```
tests/unit/test_embeddings.py::test_generate_embedding PASSED
tests/unit/test_chunking.py::test_recursive_chunking PASSED
tests/integration/test_query_endpoint.py::test_query_full_book PASSED
tests/integration/test_query_endpoint.py::test_query_selected_text PASSED

========================= 15 passed in 12.34s =========================
```

### Frontend Tests

```bash
cd frontend
npm test
```

**Expected output**:
```
PASS  src/components/ChatWidget.test.tsx
PASS  src/hooks/useChat.test.ts

Test Suites: 5 passed, 5 total
Tests:       18 passed, 18 total
Snapshots:   0 total
Time:        8.234 s
```

---

## Step 6: Development Workflow

### Making Changes to Backend

1. Edit files in `backend/src/`
2. The server will auto-reload (if using `--reload` flag)
3. Test changes via Swagger UI or curl

### Making Changes to Frontend

1. Edit files in `frontend/src/`
2. The dev server will hot-reload
3. Test changes in the browser

### Re-indexing Content

If you update the book content (edit Markdown files), re-run the indexer:

```bash
cd backend
python -m src.services.content_indexer --content-path ../physical-ai-book/docs/ --force-reindex
```

---

## Troubleshooting

### Backend Won't Start

**Issue**: `ModuleNotFoundError: No module named 'fastapi'`
**Solution**: Activate virtual environment and reinstall dependencies
```bash
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt
```

### Qdrant Connection Error

**Issue**: `QdrantException: Failed to connect to Qdrant`
**Solution**: Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
```bash
curl https://your-cluster-url.cloud.qdrant.io:6333/collections \
  -H "api-key: your-api-key"
```

### Neon Postgres Connection Error

**Issue**: `OperationalError: could not connect to server`
**Solution**: Check `DATABASE_URL` in `.env` and ensure Neon project is not suspended (free tier auto-suspends after 7 days inactivity)

### OpenRouter API Error

**Issue**: `401 Unauthorized` or `429 Too Many Requests`
**Solution**: Verify `OPENROUTER_API_KEY` and check rate limits
```bash
curl https://openrouter.ai/api/v1/auth/key \
  -H "Authorization: Bearer $OPENROUTER_API_KEY"
```

### Chat Widget Not Showing

**Issue**: Widget doesn't appear on Docusaurus site
**Solution**: Check browser console for errors. Verify `Root.tsx` is correctly importing the widget.

---

## Next Steps

1. **Implement tasks**: Run `/sp.tasks` to generate implementation tasks
2. **Deploy backend**: Deploy to AWS Lambda or Railway (see `research.md` for recommendations)
3. **Deploy frontend**: Build Docusaurus site and deploy to Vercel or GitHub Pages
4. **Monitor usage**: Track query logs in Neon Postgres

---

## Useful Commands

```bash
# Backend
cd backend
source venv/bin/activate               # Activate virtual environment
uvicorn src.main:app --reload          # Start dev server
pytest tests/ -v                        # Run tests
python -m src.services.content_indexer # Re-index content

# Frontend
cd frontend
npm start                               # Start dev server
npm test                                # Run tests
npm run build                           # Build for production

# Docusaurus
cd physical-ai-book
npm start                               # Start Docusaurus dev server
npm run build                           # Build static site
```

---

## Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Neon Postgres Documentation](https://neon.tech/docs/)
- [OpenRouter API Reference](https://openrouter.ai/docs/)
- [Docusaurus Documentation](https://docusaurus.io/docs/)
