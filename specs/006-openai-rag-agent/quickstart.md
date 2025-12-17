# Quickstart: OpenAI RAG Agent

## Overview
This guide shows how to set up and use the OpenAI RAG Agent that answers questions grounded in book content using the OpenAI Agents SDK.

## Prerequisites
- Python 3.11+
- OpenAI API key with Agent access
- Access to the existing Qdrant-backed retrieval pipeline
- FastAPI-compatible environment

## Setup
1. Install dependencies:
```bash
pip install openai fastapi uvicorn pydantic
```

2. Set environment variables:
```bash
export OPENAI_API_KEY=your_openai_api_key
export QDRANT_URL=your_qdrant_url
export QDRANT_API_KEY=your_qdrant_api_key
```

## Usage Examples

### Query the Agent
```bash
curl -X POST http://localhost:8000/v1/agent/query \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What are the main principles of humanoid robot control?"
  }'
```

### Expected Response
```json
{
  "answer": "The main principles of humanoid robot control include hierarchical control systems...",
  "citations": [
    {
      "chunk_id": "chunk_12345",
      "text_preview": "Humanoid robot control systems typically employ hierarchical architectures...",
      "url": "https://book.example.com/docs/chapter-3/lesson-2#control-systems",
      "section": "Chapter 3: Robot Control Systems",
      "relevance_score": 0.87
    }
  ],
  "confidence_score": 0.85,
  "processing_time_ms": 2450,
  "status": "success"
}
```

## Key Components
1. **Agent Service**: Orchestrates the OpenAI Agent and manages its lifecycle
2. **Retrieval Tool**: Custom tool that connects the agent to the Qdrant-backed retrieval pipeline
3. **API Layer**: FastAPI endpoints that expose agent functionality
4. **Response Validator**: Ensures responses are grounded in retrieved content

## Testing
Validate the agent responses by:
1. Submitting questions with known answers in the book
2. Verifying citations link to correct content
3. Checking that responses contain only information from retrieved chunks
4. Testing edge cases like queries with no relevant content