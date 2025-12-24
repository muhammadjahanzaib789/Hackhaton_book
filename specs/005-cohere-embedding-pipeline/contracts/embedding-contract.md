# API Contract: Cohere Embedding Pipeline

**Version**: 1.0.0 | **Date**: 2025-12-24
**Purpose**: Define the required API endpoints and data structures for the Cohere embedding system

## API Endpoints Contract

Every Cohere embedding pipeline implementation MUST conform to this API contract.

### Embedding Generation Endpoints

#### POST /api/v1/embeddings/generate
Generate embeddings for one or more text inputs using Cohere

**Request**:
```http
POST /api/v1/embeddings/generate
Content-Type: application/json

{
  "texts": ["string"],              // Array of text strings to embed
  "model": "string",                // Optional: Cohere model to use (default: best available)
  "truncate": "enum",               // Optional: "START", "END", "NONE" (default: "END")
  "batch_id": "string"              // Optional: ID for batch processing
}
```

**Response** (Success - 200):
```json
{
  "embeddings": [
    {
      "id": "string",               // Unique identifier for this embedding
      "text": "string",             // Original text (or hash if sensitive)
      "embedding": ["number"],      // Vector embedding (array of floats)
      "model": "string",            // Model used to generate embedding
      "model_version": "string",    // Version of the model
      "tokens": "number",           // Number of tokens in original text
      "metadata": {
        "source": "string",         // Source document identifier
        "created_at": "timestamp",  // When embedding was created
        "batch_id": "string"        // Batch this embedding belongs to
      }
    }
  ],
  "model_info": {
    "model": "string",              // Model name used
    "model_version": "string",      // Model version
    "input_count": "number",        // Number of texts processed
    "total_tokens": "number"        // Total tokens processed
  },
  "processing_time": "number"       // Time taken in milliseconds
}
```

**Response** (Error - 4xx/5xx):
```json
{
  "error": {
    "type": "string",               // Error type (e.g., "api_error", "validation_error")
    "message": "string",            // Human-readable error message
    "details": "object"             // Additional error details
  }
}
```

#### POST /api/v1/embeddings/batch
Create a batch job for processing multiple documents

**Request**:
```http
POST /api/v1/embeddings/batch
Content-Type: application/json

{
  "documents": [
    {
      "id": "string",               // Document identifier
      "text": "string",             // Text content to embed
      "metadata": "object"          // Optional metadata
    }
  ],
  "model": "string",                // Optional: Cohere model to use
  "batch_config": {
    "batch_size": "number",         // Number of documents per API call (default: 96)
    "concurrency": "number",        // Number of concurrent requests (default: 1)
    "retry_attempts": "number"      // Number of retry attempts (default: 3)
  }
}
```

**Response** (Success - 200):
```json
{
  "batch_id": "string",             // Unique batch job identifier
  "status": "string",               // Current status: "created", "processing", "completed", "failed"
  "total_documents": "number",      // Total number of documents in batch
  "documents_processed": "number",  // Number of documents processed so far
  "created_at": "timestamp",        // When batch was created
  "estimated_completion": "timestamp" // Estimated completion time
}
```

#### GET /api/v1/embeddings/batch/{batch_id}
Get status and results of a batch job

**Response** (Success - 200):
```json
{
  "batch_id": "string",             // Batch identifier
  "status": "string",               // Current status
  "total_documents": "number",      // Total documents in batch
  "documents_processed": "number",  // Successfully processed
  "documents_failed": "number",     // Failed to process
  "progress": "number",             // Progress percentage (0-100)
  "created_at": "timestamp",        // When batch was created
  "started_at": "timestamp",        // When processing started
  "completed_at": "timestamp",      // When processing completed (null if not done)
  "results": {                      // Results summary (only when completed)
    "average_processing_time": "number", // Average time per document (ms)
    "total_tokens_processed": "number",  // Total tokens processed
    "model_used": "string"              // Model used for processing
  }
}
```

### Vector Storage Endpoints

#### POST /api/v1/vectors/store
Store embeddings in vector database

**Request**:
```http
POST /api/v1/vectors/store
Content-Type: application/json

{
  "embeddings": [
    {
      "id": "string",               // Vector ID (will be generated if not provided)
      "values": ["number"],         // The embedding vector
      "metadata": "object"          // Metadata to store with vector
    }
  ],
  "namespace": "string",            // Optional: namespace for organization
  "upsert": "boolean"               // Whether to update existing vectors (default: true)
}
```

**Response** (Success - 200):
```json
{
  "stored_count": "number",         // Number of vectors stored
  "stored_ids": ["string"],         // IDs of stored vectors
  "namespace": "string",            // Namespace used
  "processing_time": "number"       // Time taken in milliseconds
}
```

#### POST /api/v1/vectors/search
Perform similarity search on stored vectors

**Request**:
```http
POST /api/v1/vectors/search
Content-Type: application/json

{
  "vector": ["number"],             // Query vector to find similar vectors
  "top_k": "number",                // Number of results to return (default: 10)
  "namespace": "string",            // Optional: namespace to search in
  "metadata_filter": "object",      // Optional: filter results by metadata
  "include_values": "boolean",      // Whether to include vector values in response (default: false)
  "include_metadata": "boolean"     // Whether to include metadata in response (default: true)
}
```

**Response** (Success - 200):
```json
{
  "matches": [
    {
      "id": "string",               // Vector ID
      "score": "number",            // Similarity score (0-1)
      "values": ["number"],         // Vector values (if include_values=true)
      "metadata": "object"          // Metadata (if include_metadata=true)
    }
  ],
  "namespace": "string",            // Namespace searched
  "processing_time": "number"       // Time taken in milliseconds
}
```

### Validation Endpoints

#### POST /api/v1/validation/similarity
Validate embedding quality through similarity testing

**Request**:
```http
POST /api/v1/validation/similarity
Content-Type: application/json

{
  "pairs": [
    {
      "text1": "string",            // First text in pair
      "text2": "string",            // Second text in pair
      "expected_similarity": "number" // Expected similarity score (0-1)
    }
  ],
  "model": "string"                 // Model to use for validation
}
```

**Response** (Success - 200):
```json
{
  "results": [
    {
      "text1_hash": "string",       // Hash of first text
      "text2_hash": "string",       // Hash of second text
      "expected_similarity": "number", // Expected score
      "actual_similarity": "number",  // Actual calculated score
      "difference": "number",       // Difference between expected and actual
      "validation_passed": "boolean" // Whether similarity is within threshold
    }
  ],
  "summary": {
    "total_pairs": "number",        // Total pairs tested
    "passed_pairs": "number",       // Pairs that passed validation
    "accuracy": "number",           // Overall accuracy (0-1)
    "mean_error": "number"          // Average error in similarity scores
  },
  "processing_time": "number"       // Time taken in milliseconds
}
```

### System Status Endpoints

#### GET /api/v1/health
Check system health

**Response** (Success - 200):
```json
{
  "status": "healthy",
  "timestamp": "timestamp",
  "services": {
    "cohere_api": "connected",      // Cohere API connectivity
    "vector_database": "connected", // Vector database connectivity
    "storage": "available"          // Storage system availability
  },
  "metrics": {
    "active_batch_jobs": "number",  // Number of currently running batch jobs
    "queue_size": "number",         // Number of pending batch jobs
    "embeddings_stored": "number"   // Total embeddings in database
  }
}
```

## Data Models

### Embedding Model
```javascript
{
  id: "string (UUID)",              // Unique identifier
  values: ["number"],               // The embedding vector
  model: "string",                  // Model used to generate
  model_version: "string",          // Version of the model
  tokens: "number",                 // Number of tokens in source text
  metadata: {
    source: "string",               // Source document identifier
    created_at: "timestamp",        // When embedding was created
    batch_id: "string",             // Batch job this came from
    custom: "object"                // Additional custom metadata
  },
  created_at: "timestamp",          // When stored in database
  updated_at: "timestamp"           // When last updated
}
```

### Batch Job Model
```javascript
{
  batch_id: "string (UUID)",        // Unique batch identifier
  status: "enum",                   // 'created', 'processing', 'completed', 'failed'
  total_documents: "number",        // Total documents to process
  documents_processed: "number",    // Successfully processed
  documents_failed: "number",       // Failed to process
  progress: "number",               // Percentage complete (0-100)
  model: "string",                  // Model used for processing
  created_at: "timestamp",          // When batch was created
  started_at: "timestamp",          // When processing started
  completed_at: "timestamp",        // When processing completed
  error_details: "object",          // Details about any errors
  config: {
    batch_size: "number",           // Documents per API call
    concurrency: "number",          // Concurrent requests
    retry_attempts: "number"        // Retry attempts per document
  }
}
```

### Validation Result Model
```javascript
{
  validation_id: "string (UUID)",   // Unique validation identifier
  type: "enum",                     // 'similarity', 'accuracy', 'quality'
  results: "object",                // Validation-specific results
  summary: {
    total_tests: "number",          // Total tests performed
    passed_tests: "number",         // Tests that passed
    accuracy: "number",             // Overall accuracy (0-1)
    metrics: "object"               // Additional validation metrics
  },
  created_at: "timestamp",          // When validation was run
  parameters: "object"              // Parameters used for validation
}
```

## Error Response Contract

All API endpoints MUST return errors in this format:

```json
{
  "error": {
    "type": "string",               // Error type (e.g., "validation_error", "api_error", "rate_limit")
    "message": "string",            // Human-readable error message
    "details": "object",            // Optional additional error details
    "timestamp": "timestamp",       // When error occurred
    "request_id": "string"          // ID of the failed request
  }
}
```

## Validation Checklist

Before an API implementation is considered complete:

- [ ] All endpoints return appropriate HTTP status codes
- [ ] Request/response schemas match contract exactly
- [ ] Error responses follow the error contract
- [ ] All required fields are present in responses
- [ ] Optional fields are properly handled
- [ ] API rate limiting is implemented where appropriate
- [ ] Authentication/authorization is implemented per security requirements
- [ ] All endpoints are documented with examples
- [ ] Quality Bar test: "Can a developer integrate without guessing?"

## Implementation Requirements

### Embedding Generation Requirements
- MUST handle documents up to 4096 tokens (Cohere limit)
- MUST support batch processing of up to 96 texts per API call
- MUST include proper error handling for API rate limits
- MUST track model version and metadata for each embedding

### Vector Storage Requirements
- MUST support efficient similarity search operations
- MUST allow filtering by metadata
- MUST handle large-scale vector storage (millions of vectors)
- MUST provide consistent performance regardless of scale

### Batch Processing Requirements
- MUST process documents in parallel while respecting API rate limits
- MUST provide real-time progress tracking
- MUST handle failures and retries gracefully
- MUST allow resumption of failed batch jobs