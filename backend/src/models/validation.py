"""
Data models for the validation functionality of the RAG Retrieval Validation feature.
"""

from typing import List, Dict, Optional, TYPE_CHECKING
from pydantic import BaseModel, Field, field_validator
from datetime import datetime

if TYPE_CHECKING:
    from .retrieval import RetrievalResponse


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
    results: List['RetrievalResponse'] = Field(..., description="Individual results for each query")
    metrics: Dict[str, float] = Field(..., description="Computed evaluation metrics")
    validation_latency: float = Field(..., ge=0, description="Total time for validation execution in seconds")


class MetricResult(BaseModel):
    """
    Individual metric result for validation.
    """
    name: str = Field(..., description="Name of the metric (e.g., 'precision@5')")
    value: float = Field(..., ge=0.0, le=1.0, description="Value of the metric")
    description: str = Field(..., description="Description of what the metric measures")


class ValidationResult(BaseModel):
    """
    Complete result of a validation run.
    """
    benchmark_name: str = Field(..., description="Name of the benchmark used")
    total_queries: int = Field(..., ge=0, description="Total number of queries in the benchmark")
    successful_queries: int = Field(..., ge=0, description="Number of queries that completed successfully")
    failed_queries: int = Field(..., ge=0, description="Number of queries that failed")
    metrics: List[MetricResult] = Field(..., description="Calculated metrics")
    execution_time: float = Field(..., ge=0, description="Total execution time in seconds")
    timestamp: datetime = Field(..., description="When the validation was run")