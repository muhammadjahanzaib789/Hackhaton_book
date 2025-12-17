# Data Model: RAG Retrieval Validation

## Entities

### Query
Represents a user question with parameters for retrieval

**Attributes:**
- `query_text: str` - The natural language query text (non-empty)
- `top_k: int` - Number of results to return (default: 5, min: 1, max: 50) 
- `similarity_threshold: float` - Minimum similarity score for results (default: 0.5, range: 0.0-1.0)
- `filters: Optional[Dict[str, Any]]` - Metadata filters (e.g., {'url': '...', 'section': '...'})
- `query_id: Optional[str]` - Unique identifier for the query (auto-generated if not provided)

**Validation rules:**
- query_text must not be empty or contain only whitespace
- top_k must be between 1 and 50 inclusive
- similarity_threshold must be between 0.0 and 1.0 inclusive

### QueryEmbedding
Represents the vector representation of a query

**Attributes:**
- `vector: List[float]` - 1024-dimensional embedding vector
- `model: str` - The embedding model used (e.g., 'embed-english-v3.0')
- `generation_timestamp: datetime` - When the embedding was generated

**Validation rules:**
- vector must have exactly 1024 dimensions
- model must match the expected Cohere model for consistency

### RetrievalResult
Represents a single retrieved chunk with metadata

**Attributes:**
- `chunk_id: str` - Unique identifier for the chunk in the vector database
- `text: str` - The content of the retrieved document chunk
- `url: str` - The source URL of the document
- `section: Optional[str]` - The section name (if applicable)
- `chunk_index: int` - The position of this chunk in the original document
- `similarity_score: float` - Cosine similarity score (0.0-1.0)
- `timestamp: Optional[datetime]` - When the chunk was indexed (if available)

**Validation rules:**
- similarity_score must be between 0.0 and 1.0 inclusive
- chunk_index must be non-negative
- text must not be empty

### ValidationBenchmark
Represents a test dataset for retrieval validation

**Attributes:**
- `benchmark_name: str` - Name of the benchmark set
- `question_set: List[str]` - List of query questions for testing
- `ground_truth_mappings: Dict[str, List[str]]` - Mapping of queries to expected relevant document IDs
- `metrics: List[str]` - List of metrics to compute (e.g., ['precision@5', 'recall@10', 'MRR'])

**Validation rules:**
- benchmark_name must not be empty
- question_set must not be empty
- ground_truth_mappings must have at least one mapping for each question
- metrics must be a valid list of supported metrics

### RetrievalResponse
Represents the structured response from a retrieval operation

**Attributes:**
- `query_id: str` - The ID of the original query
- `results: List[RetrievalResult]` - The ranked list of retrieval results
- `query_latency: float` - Total query latency in seconds
- `embedding_generation_time: float` - Time spent generating embeddings in seconds
- `qdrant_search_time: float` - Time spent searching in Qdrant in seconds
- `post_processing_time: float` - Time spent on post-processing in seconds
- `status: str` - Status of the retrieval operation ('success', 'partial', 'error')
- `error_message: Optional[str]` - Error message if status is 'error'

**Validation rules:**
- results must be sorted by similarity_score in descending order
- latency values must be non-negative
- status must be one of the allowed values

### BatchQueryRequest
Represents a request containing multiple queries for batch processing

**Attributes:**
- `queries: List[Query]` - List of individual query objects
- `batch_id: Optional[str]` - Unique identifier for the batch (auto-generated if not provided)

**Validation rules:**
- queries must not be empty
- queries must not exceed a reasonable batch size limit (e.g., 100 queries)

### BatchQueryResponse
Represents the response to a batch query request

**Attributes:**
- `batch_id: str` - The ID of the batch request
- `responses: List[RetrievalResponse]` - Individual responses for each query in the batch
- `batch_latency: float` - Total latency for the entire batch in seconds

**Validation rules:**
- responses length must match the original queries length
- batch_latency must be non-negative