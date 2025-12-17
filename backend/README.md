# RAG Chatbot Backend

FastAPI backend for the integrated RAG chatbot embedded in the Physical AI & Humanoid Robotics book.

## Features

- **Full-Book Query Mode**: Semantic search across entire indexed book content
- **Selected-Text Mode**: Strict constraint to answer only from user-selected passages
- **RAG Agent**: OpenAI-powered agent with retrieval tool for grounded responses
- **Smart Caching**: LRU cache for query embeddings (100 entries, 1-hour TTL)
- **Source Citations**: Automatic generation of Docusaurus navigation links
- **Performance Optimized**: <3s response time for 95% of queries
- **Comprehensive Logging**: Session tracking, query/response logging, source citations

## Tech Stack

- **Framework**: FastAPI 0.104.1
- **Embeddings**: Qwen3 Embedding 8B (768D) via OpenRouter
- **Vector DB**: Qdrant Cloud (cosine similarity)
- **Relational DB**: Neon Serverless Postgres
- **LLM**: OpenAI GPT-4 Turbo via OpenRouter
- **Chunking**: Recursive chunking (400 tokens, 80 overlap) with tiktoken

## Installation

### Prerequisites

- Python 3.11+
- Qdrant Cloud account (free tier)
- Neon Postgres account (free tier)
- OpenRouter API key

### Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Create `.env` file (see `.env.example`):
```env
OPENROUTER_API_KEY=your_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_key
DATABASE_URL=postgresql://user:pass@host/db
```

3. Run database migrations:
```bash
# Migrations run automatically on startup
python -m uvicorn src.main:app --host 0.0.0.0 --port 8000
```

4. Index book content:
```bash
curl -X POST http://localhost:8000/v1/admin/index \
  -H "Content-Type: application/json" \
  -d '{"content_path": "../physical-ai-book/docs", "force_reindex": false}'
```

## API Endpoints

### Health Check
```http
GET /v1/health
```

Returns status of Qdrant, Neon Postgres, and OpenRouter.

### Index Content (Admin)
```http
POST /v1/admin/index
Content-Type: application/json

{
  "content_path": "/path/to/book/docs",
  "force_reindex": false
}
```

Indexes Markdown files from the specified directory.

### Query Chatbot
```http
POST /v1/query
Content-Type: application/json

{
  "session_id": "optional-uuid",
  "query": "What are the main components of a humanoid robot?",
  "mode": "full_book",
  "selected_text": null
}
```

**Modes**:
- `full_book`: Searches entire indexed content
- `selected_text`: Only uses the provided `selected_text` (must be ≥10 chars)

**Response**:
```json
{
  "session_id": "uuid",
  "query_id": "uuid",
  "answer": "Generated answer...",
  "sources": [
    {
      "chunk_id": "uuid",
      "chapter_name": "Chapter 3",
      "section_name": "Robot Control",
      "relevance_score": 0.89,
      "text_preview": "...",
      "excerpt": "...",
      "link": "/docs/chapter-03/lesson-01#robot-control"
    }
  ],
  "mode": "full_book",
  "processing_time_ms": 1250
}
```

### Query RAG Agent
```http
POST /v1/agent/query
Content-Type: application/json

{
  "query": "What are the key principles of embodied cognition in physical AI?",
  "max_tokens": 500,
  "temperature": 0.7,
  "metadata": {
    "user_id": "optional-user-id",
    "session_id": "optional-session-id"
  }
}
```

**Parameters**:
- `query`: The question to ask the RAG Agent (required)
- `max_tokens`: Maximum number of tokens in the response (optional, default: 500, range: 1-2000)
- `temperature`: Controls randomness in the response (optional, default: 0.7, range: 0.0-2.0)
- `metadata`: Additional metadata for tracking and analytics (optional)

**Response**:
```json
{
  "answer": "The RAG Agent provides grounded responses based on retrieved information...",
  "citations": [
    {
      "id": "citation_1",
      "source_document": "physical_ai_book.pdf",
      "chunk_text": "Retrieved text chunk...",
      "relevance_score": 0.85,
      "url": "/documents/physical_ai_book.pdf#page=15",
      "document_title": "Physical AI Fundamentals",
      "page_number": 15,
      "section_title": "Embodied Cognition",
      "position_in_document": 15,
      "context_before": "Previous context...",
      "context_after": "Following context...",
      "citation_format": "APA",
      "access_date": "2024-01-01T10:00:00"
    }
  ],
  "query": "What are the key principles of embodied cognition?",
  "tokens_used": 150,
  "confidence": 0.85
}
```

### Validate Agent Response
```http
POST /v1/agent/validate
Content-Type: application/json

{
  "query": "What are the key principles of embodied cognition?",
  "response": "The key principles of embodied cognition...",
  "citations": [
    {
      "id": "citation_1",
      "source_document": "physical_ai_book.pdf",
      "chunk_text": "Retrieved text chunk...",
      "relevance_score": 0.85
    }
  ]
}
```

**Response**:
```json
{
  "grounding_score": 0.85,
  "citation_accuracy": 0.9,
  "is_valid": true,
  "feedback": "Response appears to be well-grounded with accurate citations.",
  "timestamp": "2024-01-01T10:00:00"
}
```

### Agent Health Check
```http
GET /v1/agent/health
```

**Response**:
```json
{
  "status": "healthy",
  "timestamp": "2024-01-01T10:00:00",
  "service": "rag-agent"
}
```

### Submit Agent Feedback
```http
POST /v1/agent/feedback
Content-Type: application/json

{
  "query": "What are the key principles of embodied cognition?",
  "response": "The key principles of embodied cognition...",
  "feedback": "The response was helpful and accurate.",
  "rating": 5
}
```

**Response**:
```json
{
  "status": "received",
  "message": "Thank you for your feedback. This will help improve future responses.",
  "feedback_id": "feedback_1234567890"
}
```

## Project Structure

```
backend/
├── src/
│   ├── api/
│   │   ├── routes/
│   │   │   ├── health.py       # Health check endpoint
│   │   │   ├── index.py        # Content indexing endpoint
│   │   │   └── query.py        # Main query endpoint
│   │   ├── agent.py            # RAG Agent API endpoints
│   │   ├── retrieval.py        # Retrieval API endpoints
│   │   ├── validation.py       # Validation API endpoints
│   │   └── middleware.py       # Logging & rate limiting
│   ├── agents/                  # OpenAI Agents SDK components
│   │   ├── __init__.py         # Agent module initialization
│   │   ├── rag_agent.py        # Core RAG Agent implementation
│   │   └── retrieval_tool.py   # Retrieval tool for agent
│   ├── db/
│   │   ├── neon_client.py      # Postgres connection pooling
│   │   ├── qdrant_client.py    # Vector DB operations
│   │   └── migrations/         # SQL migration files
│   ├── models/                  # Pydantic models
│   │   ├── agent.py            # Agent request/response models
│   │   ├── chunk.py
│   │   ├── session.py
│   │   ├── query.py
│   │   ├── response.py
│   │   └── citation.py
│   ├── services/
│   │   ├── content_indexer.py  # MD parsing & chunking
│   │   ├── embeddings.py       # Qwen3 embedding generation
│   │   ├── retrieval.py        # Vector search + caching
│   │   ├── llm_generation.py   # Answer generation
│   │   ├── agent_service.py    # Agent orchestration service
│   │   └── chat_logger.py      # Session/query/response logging
│   ├── utils/
│   │   ├── agent_logging.py    # Agent-specific logging
│   │   └── agent_errors.py     # Agent-specific error handling
│   ├── config.py                # Pydantic Settings
│   └── main.py                  # FastAPI app
├── requirements.txt
├── Dockerfile
└── .env.example
```

## Key Services

### ContentIndexer (`src/services/content_indexer.py`)
- Scans directory for Markdown files
- Parses frontmatter (title, section, page)
- Recursive chunking: preserves code blocks, splits by paragraphs/sentences
- Generates embeddings via OpenRouter
- Stores vectors in Qdrant, metadata in Neon Postgres

### RetrievalService (`src/services/retrieval.py`)
- **Full-book mode**: Semantic search in Qdrant with query embedding
- **Selected-text mode**: Creates in-memory chunk (no Qdrant search)
- LRU cache for query embeddings (100 entries, 1-hour TTL)
- Enriches results with metadata from Neon Postgres

### LLMGenerationService (`src/services/llm_generation.py`)
- Mode-specific prompts:
  - **Full-book**: Uses all retrieved chunks
  - **Selected-text**: Strict guardrail - "ONLY answer using provided selected text"
- 5-second timeout enforcement
- Temperature: 0.7, Max tokens: 500

### AgentService (`src/services/agent_service.py`)
- Core orchestration for the RAG Agent system
- Integration with OpenAI Agents SDK
- Query processing with retrieval tool integration
- Response generation with proper grounding validation
- Source citation formatting and relevance scoring
- Performance metrics and monitoring

### ChatLoggerService (`src/services/chat_logger.py`)
- Session management (create, update status)
- Query/response logging
- Source citation tracking with Docusaurus link generation
- Citation ordering by relevance

## Configuration

All configuration via environment variables (see `.env.example`):

| Variable | Description | Default |
|----------|-------------|---------|
| `OPENROUTER_API_KEY` | OpenRouter API key | Required |
| `OPENROUTER_BASE_URL` | OpenRouter base URL | `https://openrouter.ai/api/v1` |
| `EMBEDDING_MODEL` | Embedding model | `qwen/qwen3-embedding-8b` |
| `LLM_MODEL` | LLM model | `openai/gpt-4-turbo-preview` |
| `QDRANT_URL` | Qdrant cluster URL | Required |
| `QDRANT_API_KEY` | Qdrant API key | Required |
| `DATABASE_URL` | Neon Postgres connection string | Required |
| `CHUNK_SIZE` | Token count per chunk | 400 |
| `CHUNK_OVERLAP` | Overlap between chunks | 80 |
| `CORS_ORIGINS` | Allowed origins (comma-separated) | `http://localhost:3000` |
| `OPENAI_API_KEY` | OpenAI API key for agent functionality | Required for agent features |
| `OPENAI_AGENT_ID` | OpenAI Assistant ID for the RAG agent | Optional |
| `OPENAI_MODEL` | OpenAI model for agent operations | `gpt-4-turbo-preview` |

## Database Schema

### book_content_chunks
- `chunk_id` (UUID, PK)
- `text`, `chapter_name`, `section_name`, `page_number`
- `document_path`, `chunk_index`, `token_count`
- Vectors stored in Qdrant (768D)

### chat_sessions
- `session_id` (UUID, PK)
- `mode` (full_book | selected_text)
- `status` (active | ended | abandoned)
- `started_at`, `ended_at`

### queries
- `query_id` (UUID, PK)
- `session_id` (FK), `query_text`, `mode`, `selected_text`

### responses
- `response_id` (UUID, PK)
- `query_id` (FK), `answer_text`, `model_name`, `processing_time_ms`

### source_citations
- `citation_id` (UUID, PK)
- `response_id` (FK), `chunk_id` (FK)
- `chapter_name`, `section_name`, `page_number`, `relevance_score`
- `excerpt`, `link`, `citation_order`

## Testing

### Run Unit Tests
```bash
pytest tests/ -v
```

### Manual API Testing
```bash
# Health check
curl http://localhost:8000/v1/health

# Agent health check
curl http://localhost:8000/v1/agent/health

# Query (full-book mode)
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "mode": "full_book"}'

# Query (selected-text mode)
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What does this mean?",
    "mode": "selected_text",
    "selected_text": "ROS 2 uses a distributed architecture..."
  }'

# Query RAG Agent
curl -X POST http://localhost:8000/v1/agent/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of embodied cognition?",
    "max_tokens": 500,
    "temperature": 0.7
  }'

# Validate agent response
curl -X POST http://localhost:8000/v1/agent/validate \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of embodied cognition?",
    "response": "The key principles of embodied cognition...",
    "citations": [
      {
        "id": "citation_1",
        "source_document": "physical_ai_book.pdf",
        "chunk_text": "Retrieved text chunk...",
        "relevance_score": 0.85
      }
    ]
  }'

# Submit agent feedback
curl -X POST http://localhost:8000/v1/agent/feedback \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of embodied cognition?",
    "response": "The key principles of embodied cognition...",
    "feedback": "The response was helpful and accurate.",
    "rating": 5
  }'
```

### Load Testing
```bash
# Install locust
pip install locust

# Run load test (target: 95% < 3s latency)
locust -f tests/load_test.py --host http://localhost:8000
```

## Deployment

### Docker
```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

### AWS Lambda (Production)
See `../specs/001-integrated-rag-chatbot/deployment.md` for:
- Lambda function creation with container image
- API Gateway configuration
- Environment variable setup
- Mangum adapter usage

## Performance

- **Target**: <3s response time for 95% of queries
- **Optimizations**:
  - LRU cache for query embeddings (saves ~200ms per cache hit)
  - Connection pooling (2-10 connections)
  - 5-second timeout on LLM requests
  - Async operations throughout

## Troubleshooting

### "Service unavailable" errors
Check health endpoint: `curl http://localhost:8000/v1/health`

### No relevant content found
- Verify indexing completed: Check logs for "chunks indexed" count
- Check Qdrant collection: `curl {QDRANT_URL}/collections/book_chunks`

### Slow responses
- Check OpenRouter rate limits
- Review `processing_time_ms` in responses
- Monitor Qdrant search latency in health check

## License

MIT

## Support

- API Docs: http://localhost:8000/docs (auto-generated by FastAPI)
- Specification: `../specs/001-integrated-rag-chatbot/spec.md`
- Tasks: `../specs/001-integrated-rag-chatbot/tasks.md`
