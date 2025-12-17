# API Contracts

This directory contains API contracts for the RAG Chatbot backend.

## Files

- **openapi.yaml**: OpenAPI 3.0 specification for the REST API
  - Endpoints: `/health`, `/query`, `/index`
  - Request/response schemas
  - Error responses
  - Examples

## Viewing the API Documentation

You can visualize the OpenAPI spec using:

1. **Swagger UI**: https://editor.swagger.io/ (paste the YAML content)
2. **Redoc**: https://redocly.github.io/redoc/ (paste the YAML URL)
3. **Local Swagger UI** (after starting the FastAPI server):
   ```bash
   # Start the backend server
   cd backend
   python -m uvicorn src.main:app --reload

   # Open in browser
   open http://localhost:8000/docs
   ```

## Key Endpoints

### POST /v1/query
Submit a question to the chatbot. Returns an answer with source citations.

**Request**:
```json
{
  "query": "What are the key components of a robotic control system?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Response**:
```json
{
  "query_id": "660e8400-e29b-41d4-a716-446655440001",
  "response_text": "A robotic control system consists of...",
  "citations": [
    {
      "chapter_name": "Chapter 1: Introduction to ROS 2",
      "relevance_score": 0.92,
      "link": "/docs/module-01/lesson-01#ros2-core-concepts"
    }
  ],
  "mode": "full_book",
  "processing_time_ms": 2340
}
```

### POST /v1/index (Admin)
Index book content for semantic search.

**Request**:
```json
{
  "content_path": "physical-ai-book/docs/",
  "force_reindex": false
}
```

**Response**:
```json
{
  "status": "success",
  "chunks_indexed": 1247,
  "processing_time_ms": 45320
}
```

## Error Handling

All endpoints return standard error responses:

```json
{
  "error": "INVALID_REQUEST",
  "message": "Query text must be between 5 and 500 characters",
  "details": {
    "field": "query",
    "constraint": "minLength"
  }
}
```

## Rate Limits

- **OpenRouter**: 1,000 requests/day (with $10 credits)
- **API Rate Limiting**: TBD (to be implemented in middleware)

## Idempotency

- **POST /query**: Not idempotent (each query creates a new log entry)
- **POST /index**: Idempotent if `force_reindex=true` (same chunks are re-indexed)

## Versioning

API version is included in the URL path (`/v1/`). Future versions will be `/v2/`, etc.
