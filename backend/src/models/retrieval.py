"""
Data models for the RAG Retrieval Validation feature.

These models define the structure for queries, results, and responses
according to the specification.
"""

from typing import List, Optional, Dict, Any
from datetime import datetime
from pydantic import BaseModel, Field, field_validator
from enum import Enum


class Query(BaseModel):
    """
    Represents a user question with parameters for retrieval.
    """
    query_text: str = Field(..., description="The natural language query text")
    top_k: int = Field(5, ge=1, le=50, description="Number of results to return")
    similarity_threshold: float = Field(0.5, ge=0.0, le=1.0, description="Minimum similarity score for results")
    filters: Optional[Dict[str, Any]] = Field(None, description="Metadata filters (e.g., {'url': '...', 'section': '...'})")
    query_id: Optional[str] = Field(None, description="Unique identifier for the query (auto-generated if not provided)")

    @field_validator('query_text')
    @classmethod
    def validate_query_text(cls, v):
        if not v or not v.strip():
            raise ValueError('Query text must not be empty')
        return v.strip()


class QueryEmbedding(BaseModel):
    """
    Represents the vector representation of a query.
    """
    vector: List[float] = Field(..., description="1024-dimensional embedding vector")
    model: str = Field(..., description="The embedding model used (e.g., 'embed-english-v3.0')")
    generation_timestamp: datetime = Field(..., description="When the embedding was generated")

    @field_validator('vector')
    @classmethod
    def validate_vector_dimensions(cls, v):
        if len(v) != 1024:
            raise ValueError('Vector must have exactly 1024 dimensions')
        return v


class RetrievalResult(BaseModel):
    """
    Represents a single retrieved chunk with metadata.
    """
    chunk_id: str = Field(..., description="Unique identifier for the chunk in the vector database")
    text: str = Field(..., description="The content of the retrieved document chunk")
    url: str = Field(..., description="The source URL of the document")
    section: Optional[str] = Field(None, description="The section name (if applicable)")
    chunk_index: int = Field(..., ge=0, description="The position of this chunk in the original document")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Cosine similarity score (0.0-1.0)")
    timestamp: Optional[datetime] = Field(None, description="When the chunk was indexed (if available)")

    @field_validator('similarity_score')
    @classmethod
    def validate_similarity_score(cls, v):
        if not 0.0 <= v <= 1.0:
            raise ValueError('Similarity score must be between 0.0 and 1.0')
        return v

    @field_validator('text')
    @classmethod
    def validate_text(cls, v):
        if not v:
            raise ValueError('Text must not be empty')
        return v


class RetrievalResponse(BaseModel):
    """
    Represents the structured response from a retrieval operation.
    """
    query_id: str = Field(..., description="The ID of the original query")
    results: List[RetrievalResult] = Field(..., description="The ranked list of retrieval results")
    query_latency: float = Field(..., ge=0, description="Total query latency in seconds")
    embedding_generation_time: float = Field(..., ge=0, description="Time spent generating embeddings in seconds")
    qdrant_search_time: float = Field(..., ge=0, description="Time spent searching in Qdrant in seconds")
    post_processing_time: float = Field(..., ge=0, description="Time spent on post-processing in seconds")
    status: str = Field(..., pattern=r"^(success|partial|error)$", description="Status of the retrieval operation")
    error_message: Optional[str] = Field(None, description="Error message if status is 'error'")

    @field_validator('results')
    @classmethod
    def validate_results_order(cls, v):
        # Verify results are sorted by similarity_score in descending order
        if len(v) > 1:
            for i in range(len(v) - 1):
                if v[i].similarity_score < v[i + 1].similarity_score:
                    raise ValueError('Results must be sorted by similarity_score in descending order')
        return v


class BatchQueryRequest(BaseModel):
    """
    Represents a request containing multiple queries for batch processing.
    """
    queries: List[Query] = Field(..., min_length=1, max_length=100, description="List of individual query objects")
    batch_id: Optional[str] = Field(None, description="Unique identifier for the batch (auto-generated if not provided)")


class BatchQueryResponse(BaseModel):
    """
    Represents the response to a batch query request.
    """
    batch_id: str = Field(..., description="The ID of the batch request")
    responses: List[RetrievalResponse] = Field(..., description="Individual responses for each query in the batch")
    batch_latency: float = Field(..., ge=0, description="Total latency for the entire batch in seconds")


class ValidationBenchmark(BaseModel):
    """
    Represents a test dataset for retrieval validation.
    """
    benchmark_name: str = Field(..., min_length=1, description="Name of the benchmark set")
    question_set: List[str] = Field(..., min_length=1, description="List of query questions for testing")
    ground_truth_mappings: Dict[str, List[str]] = Field(..., description="Mapping of queries to expected relevant document IDs")
    metrics: List[str] = Field(..., min_length=1, description="List of metrics to compute (e.g., ['precision@5', 'recall@10', 'MRR'])")

    @field_validator('ground_truth_mappings')
    @classmethod
    def validate_ground_truth_mappings(cls, v, values):
        if 'question_set' in values.data:
            questions = values.data['question_set']
            for question in questions:
                if question not in v:
                    raise ValueError(f"Ground truth mapping missing for question: {question}")
        return v


class ValidationResponse(BaseModel):
    """
    Represents the response for validation operations.
    """
    benchmark_name: str = Field(..., description="Name of the benchmark set")
    results: List[RetrievalResponse] = Field(..., description="Individual results for each query")
    metrics: Dict[str, float] = Field(..., description="Computed evaluation metrics")
    validation_latency: float = Field(..., ge=0, description="Total time for validation execution in seconds")


class ErrorResponse(BaseModel):
    """
    Represents an error response.
    """
    error: str = Field(..., description="Error message")
    error_code: str = Field(..., description="Error code")
    timestamp: datetime = Field(..., description="When the error occurred")