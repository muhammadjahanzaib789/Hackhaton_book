# Quickstart Guide: RAG Retrieval Validation

## Overview
This guide provides setup instructions and usage examples for the RAG retrieval validation feature, which enables semantic search against the Qdrant vector database using Cohere embeddings.

## Prerequisites
- Python 3.11+
- Access to Cohere API (API key with embed-english-v3.0 access)
- Access to Qdrant Cloud (URL and API key)
- The ingestion pipeline from Feature 005 must be completed and the Qdrant collection populated

## Setup

### 1. Environment Configuration
Copy the environment template and configure your variables:

```bash
cp backend/.env.example backend/.env
```

Edit `backend/.env` to include:
```bash
COHERE_API_KEY=your-cohere-api-key
QDRANT_URL=your-qdrant-cloud-url
QDRANT_API_KEY=your-qdrant-api-key
```

### 2. Install Dependencies
Ensure the backend dependencies are installed:

```bash
cd backend
pip install -r requirements.txt
```

### 3. Verify Qdrant Connection
Make sure your Qdrant instance is available and the collection is populated with embeddings:

```bash
# Test connection with Python
python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='YOUR_URL', api_key='YOUR_API_KEY'); print(client.get_collections())"
```

## Usage Examples

### 1. Running a Semantic Search Query
Submit a natural language query to retrieve relevant document chunks:

```bash
curl -X POST http://localhost:8000/retrieval/query \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What is gradient descent?",
    "top_k": 5,
    "similarity_threshold": 0.5,
    "filters": {
      "url": "https://book-url.com/docs/chapter-3"
    }
  }'
```

Response:
```json
{
  "query_id": "query-12345",
  "results": [
    {
      "chunk_id": "chunk-abc123",
      "text": "Gradient descent is an optimization algorithm...",
      "url": "https://book-url.com/docs/chapter-3/optimization",
      "section": "algorithms",
      "chunk_index": 2,
      "similarity_score": 0.87,
      "timestamp": "2023-10-20T14:30:00Z"
    }
  ],
  "query_latency": 1.24,
  "embedding_generation_time": 0.56,
  "qdrant_search_time": 0.42,
  "post_processing_time": 0.26,
  "status": "success"
}
```

### 2. Running Batch Queries
Process multiple queries in a single request:

```bash
curl -X POST http://localhost:8000/retrieval/batch \
  -H "Content-Type: application/json" \
  -d '{
    "queries": [
      {
        "query_text": "What is gradient descent?",
        "top_k": 5
      },
      {
        "query_text": "Explain neural networks",
        "top_k": 3
      }
    ]
  }'
```

### 3. Running Validation Tests
Evaluate the retrieval pipeline with a benchmark dataset:

```bash
curl -X POST http://localhost:8000/validation/run \
  -H "Content-Type: application/json" \
  -d '{
    "benchmark_name": "chapter-3-validation",
    "question_set": [
      "What is gradient descent?",
      "Explain backpropagation"
    ],
    "ground_truth_mappings": {
      "What is gradient descent?": ["chunk-abc123", "chunk-def456"],
      "Explain backpropagation": ["chunk-ghi789"]
    },
    "metrics": ["precision@5", "recall@10", "MRR"]
  }'
```

## Key Components

### Services
- `embedding_service.py`: Handles generation of query embeddings using Cohere API
- `qdrant_service.py`: Manages interaction with Qdrant vector database
- `retrieval_service.py`: Core business logic for the retrieval process

### API Endpoints
- `POST /retrieval/query`: Single semantic search query
- `POST /retrieval/batch`: Batch semantic search queries
- `POST /validation/run`: Run validation benchmark tests

### Data Models
- `Query`: Represents a search query with parameters
- `RetrievalResult`: Represents a single retrieved document chunk
- `RetrievalResponse`: Structured response containing ranked results
- `ValidationBenchmark`: Represents a validation test dataset

## Testing

### Unit Tests
Run unit tests for the retrieval components:
```bash
cd backend
python -m pytest tests/unit/test_retrieval.py -v
```

### Integration Tests
Run integration tests to verify end-to-end functionality:
```bash
cd backend
python -m pytest tests/integration/test_retrieval.py -v
```

## Troubleshooting

### Common Issues
1. **"Invalid API Key" Error**: Verify your Cohere and Qdrant API keys are correct and have appropriate permissions
2. **No Results Returned**: Check that the Qdrant collection was properly populated by the ingestion pipeline
3. **High Latency**: Consider adjusting the similarity threshold or top_k values to reduce the search space

### Performance Tips
- Use metadata filters to reduce the search space when possible
- Monitor embedding generation time as it's often the slowest component
- Consider caching results for frequently asked questions