# RAG Retrieval and Validation Feature Guide

This guide explains how to use the RAG Retrieval and Validation feature that was implemented.

## Overview

The RAG Retrieval and Validation feature provides:
- Semantic search using Cohere embeddings and Qdrant vector database
- Metadata-aware filtering for targeted retrieval
- Comprehensive validation and metrics computation
- Performance monitoring with detailed timing breakdowns

## API Endpoints

### Retrieval Endpoints

**Single Query Retrieval**
- Endpoint: `POST /v1/retrieval/query`
- Purpose: Execute semantic search against vector database
- Parameters:
  - `query_text`: Natural language query (required)
  - `top_k`: Number of results to return (1-50, default: 5)
  - `similarity_threshold`: Minimum similarity score (0.0-1.0, default: 0.5)
  - `filters`: Metadata filters (optional, e.g., {"section": "Chapter 4"})

**Batch Query Retrieval**
- Endpoint: `POST /v1/retrieval/batch`
- Purpose: Execute multiple queries in a single request
- Parameters:
  - `queries`: Array of query objects with the same parameters as single query

### Validation Endpoints

**Run Validation Benchmark**
- Endpoint: `POST /v1/validation/run`
- Purpose: Execute validation tests and compute metrics
- Parameters:
  - `benchmark_name`: Name of the benchmark
  - `question_set`: Array of test questions
  - `ground_truth_mappings`: Mapping of questions to expected document IDs
  - `metrics`: Array of metrics to compute (e.g., ["precision@5", "recall@5", "MRR"])

**Calculate Metrics**
- Endpoint: `POST /v1/validation/metrics`
- Purpose: Calculate metrics for existing retrieval results
- Parameters: benchmark definition and retrieval results

## Example Usage

### Python Client Example

```python
import httpx

async def example_usage():
    async with httpx.AsyncClient(base_url="http://localhost:8000", timeout=30.0) as client:
        # Single retrieval query
        response = await client.post("/v1/retrieval/query", json={
            "query_text": "What is gradient descent?",
            "top_k": 5,
            "similarity_threshold": 0.5
        })

        print("Retrieval response:", response.json())

        # Validation benchmark
        validation_response = await client.post("/v1/validation/run", json={
            "benchmark_name": "test_benchmark",
            "question_set": ["What is gradient descent?", "How does backpropagation work?"],
            "ground_truth_mappings": {
                "What is gradient descent?": ["chunk_1", "chunk_2"],
                "How does backpropagation work?": ["chunk_3", "chunk_4"]
            },
            "metrics": ["precision@5", "recall@5", "MRR"]
        })

        print("Validation response:", validation_response.json())
```

### cURL Examples

**Simple Retrieval Query**
```bash
curl -X POST http://localhost:8000/v1/retrieval/query \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "Explain neural networks",
    "top_k": 5,
    "similarity_threshold": 0.6
  }'
```

**Retrieval with Metadata Filtering**
```bash
curl -X POST http://localhost:8000/v1/retrieval/query \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What are activation functions?",
    "top_k": 3,
    "similarity_threshold": 0.5,
    "filters": {
      "section": "Chapter 3"
    }
  }'
```

## Response Format

### Retrieval Response
```json
{
  "query_id": "retrieval-1234567890",
  "results": [
    {
      "chunk_id": "chunk_123",
      "text": "The content of the retrieved document chunk...",
      "url": "https://source-document.com",
      "section": "Chapter 3",
      "chunk_index": 0,
      "similarity_score": 0.85
    }
  ],
  "query_latency": 0.234,
  "embedding_generation_time": 0.1,
  "qdrant_search_time": 0.1,
  "post_processing_time": 0.034,
  "status": "success",
  "error_message": null
}
```

### Validation Response
```json
{
  "benchmark_name": "test_benchmark",
  "results": [...],  // Array of retrieval responses
  "metrics": {
    "precision@5": 0.8,
    "recall@5": 0.75,
    "MRR": 0.85
  },
  "validation_latency": 2.45
}
```

## Configuration

The feature uses the following environment variables (set in `.env`):
- `COHERE_API_KEY`: Cohere API key for embedding generation
- `QDRANT_URL`: Qdrant cloud instance URL
- `QDRANT_API_KEY`: Qdrant API key
- `QDRANT_COLLECTION_NAME`: Name of the vector collection (default: "rag_embedding")
- `COHERE_MODEL`: Embedding model to use (default: "embed-english-v3.0")

## Performance Metrics

The system tracks detailed performance metrics:
- **Query Latency**: Total time for the entire query operation
- **Embedding Generation Time**: Time to create embeddings from text
- **Qdrant Search Time**: Time to search the vector database
- **Post-processing Time**: Time for result formatting and filtering
- **Validation Latency**: Total time for validation benchmark execution

## Error Handling

The API provides structured error responses with appropriate HTTP status codes:
- 400 Bad Request: Invalid query parameters or validation errors
- 500 Internal Server Error: System-level failures
- Detailed error messages with error codes for debugging

## Integration with Existing System

The RAG Retrieval and Validation feature integrates seamlessly with the existing backend:
- API routes are registered in `main.py`
- Uses the same configuration system
- Follows the same error handling patterns
- Compatible with existing authentication and middleware