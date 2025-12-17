# RAG Chatbot Integration Guide

This guide explains how to integrate the completed RAG chatbot into the Physical AI & Humanoid Robotics book website.

## Prerequisites

- Node.js 18+ installed
- Python 3.11+ installed
- Backend services configured (Qdrant, Neon Postgres, OpenRouter API key)

## Step 1: Install Dependencies

### Backend
```bash
cd backend
pip install -r requirements.txt
```

### Frontend
```bash
cd frontend
npm install
```

### Docusaurus
```bash
cd physical-ai-book
npm install
```

## Step 2: Configure Environment Variables

### Backend Configuration

Create `backend/.env`:
```env
# OpenRouter API
OPENROUTER_API_KEY=your_openrouter_api_key
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
EMBEDDING_MODEL=qwen/qwen3-embedding-8b
LLM_MODEL=openai/gpt-4-turbo-preview

# Qdrant Cloud
QDRANT_URL=https://your-qdrant-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key

# Neon Postgres
DATABASE_URL=postgresql://user:password@host/dbname?sslmode=require

# API Configuration
ENVIRONMENT=development
CORS_ORIGINS=http://localhost:3000,http://localhost:8000
CHUNK_SIZE=400
CHUNK_OVERLAP=80
```

### Frontend Configuration

Create `physical-ai-book/.env`:
```env
REACT_APP_API_URL=http://localhost:8000/v1
```

## Step 3: Run Database Migrations

```bash
cd backend
python -m src.db.neon_client  # This will run migrations on startup
```

## Step 4: Index Book Content

Start the backend server:
```bash
cd backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Index the book content:
```bash
curl -X POST http://localhost:8000/v1/admin/index \
  -H "Content-Type: application/json" \
  -d '{
    "content_path": "../physical-ai-book/docs",
    "force_reindex": false
  }'
```

Expected output: ~1000 chunks indexed successfully

## Step 5: Build Frontend Widget

```bash
cd frontend
npm run build
```

This creates a production build in `frontend/dist/`.

## Step 6: Start Docusaurus

```bash
cd physical-ai-book
npm start
```

The site will be available at http://localhost:3000 with the chat widget integrated.

## Step 7: Test the Integration

### Test Full-Book Mode:
1. Open http://localhost:3000
2. Click the chat button (bottom-right)
3. Ask: "What are the main components of a humanoid robot?"
4. Verify:
   - Answer appears within 2-3 seconds
   - Sources are displayed with chapter names
   - Citations are clickable and navigate to book sections

### Test Selected-Text Mode:
1. Highlight any paragraph in the book
2. The chat button shows a notification badge "!"
3. Click the chat button
4. "Ask About Selection" mode is auto-selected
5. Ask: "Explain this in simpler terms"
6. Verify:
   - Answer only references the selected text
   - System respects the guardrail constraint

## API Endpoints

### Health Check
```bash
curl http://localhost:8000/v1/health
```

### Query Chatbot (Full-Book Mode)
```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "mode": "full_book"
  }'
```

### Query Chatbot (Selected-Text Mode)
```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What does this mean?",
    "mode": "selected_text",
    "selected_text": "ROS 2 uses a distributed architecture with nodes..."
  }'
```

## Production Deployment

See `specs/001-integrated-rag-chatbot/deployment.md` for:
- AWS Lambda deployment steps
- Qdrant Cloud production setup
- Neon Postgres production configuration
- Environment variable management
- CORS configuration for production domain

## Troubleshooting

### Chat widget not appearing:
- Check browser console for errors
- Verify `Root.tsx` is in `physical-ai-book/src/theme/`
- Ensure CSS file is imported correctly

### "Service unavailable" errors:
- Check backend is running: `curl http://localhost:8000/v1/health`
- Verify database connections in health check response
- Check OpenRouter API key is valid

### No relevant content found:
- Verify content was indexed: Check logs for "chunks indexed" count
- Try different query phrasing
- Check Qdrant collection has vectors: `curl http://qdrant-url/collections/book_chunks`

### Slow responses (>3s):
- Check OpenRouter rate limits
- Verify Qdrant search performance
- Review processing_time_ms in responses to identify bottleneck

## Architecture Overview

```
┌──────────────────┐
│  Docusaurus Site │
│   (Port 3000)    │
└────────┬─────────┘
         │
         │ React Widget
         │
┌────────▼─────────┐
│   FastAPI Backend│
│   (Port 8000)    │
└────────┬─────────┘
         │
    ┌────┴────┐
    │         │
┌───▼──┐  ┌──▼────┐
│Qdrant│  │ Neon  │
│(768D)│  │Postgres│
└──────┘  └───────┘
    │
┌───▼──────────┐
│  OpenRouter  │
│ (Qwen + GPT4)│
└──────────────┘
```

## File Structure

```
loop/
├── backend/
│   ├── src/
│   │   ├── api/routes/
│   │   │   ├── health.py      # GET /v1/health
│   │   │   ├── index.py       # POST /v1/admin/index
│   │   │   └── query.py       # POST /v1/query
│   │   ├── services/
│   │   │   ├── content_indexer.py
│   │   │   ├── embeddings.py
│   │   │   ├── retrieval.py
│   │   │   ├── llm_generation.py
│   │   │   └── chat_logger.py
│   │   ├── db/
│   │   │   ├── neon_client.py
│   │   │   ├── qdrant_client.py
│   │   │   └── migrations/
│   │   ├── models/
│   │   └── main.py
│   └── requirements.txt
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatWidget.tsx
│   │   │   ├── ChatMessage.tsx
│   │   │   ├── ChatInput.tsx
│   │   │   ├── SourceCitation.tsx
│   │   │   └── LoadingIndicator.tsx
│   │   ├── hooks/
│   │   │   ├── useChat.ts
│   │   │   └── useSelection.ts
│   │   ├── services/
│   │   │   └── api.ts
│   │   ├── types/
│   │   │   └── chat.ts
│   │   └── styles/
│   │       └── chat-widget.css
│   └── package.json
└── physical-ai-book/
    ├── docs/               # Book content (to be indexed)
    ├── src/theme/
    │   └── Root.tsx        # Chat widget integration
    └── .env                # API URL configuration
```

## Next Steps

1. Complete Phase 9: Deploy to production (AWS Lambda + Qdrant + Neon)
2. Complete Phase 10: Add rate limiting, session cleanup, final polish
3. Create comprehensive README files for backend and frontend
4. Run end-to-end testing scenarios per `tasks.md`

## Support

For issues or questions, refer to:
- Backend API docs: http://localhost:8000/docs (FastAPI auto-generated)
- Specification: `specs/001-integrated-rag-chatbot/spec.md`
- Architecture: `specs/001-integrated-rag-chatbot/plan.md`
- Tasks checklist: `specs/001-integrated-rag-chatbot/tasks.md`
