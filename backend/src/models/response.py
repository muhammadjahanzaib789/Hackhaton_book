"""
Response model for chatbot answers.
Based on data-model.md specification.
"""

from datetime import datetime
from typing import Optional, List
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator


class Response(BaseModel):
    """
    Represents the chatbot's answer to a query.

    Storage: Neon Postgres
    """

    response_id: UUID = Field(default_factory=uuid4, description="Unique response identifier")
    query_id: UUID = Field(..., description="Reference to query (1-to-1)")
    response_text: str = Field(..., min_length=50, max_length=2000, description="The generated answer")
    confidence_score: Optional[float] = Field(
        None, ge=0.0, le=1.0, description="Confidence score from LLM"
    )
    model_used: str = Field(..., max_length=100, description="LLM model name")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Response generation time")
    token_count: Optional[int] = Field(None, ge=0, description="Number of tokens in response")

    @field_validator("confidence_score")
    @classmethod
    def validate_confidence_score(cls, v):
        """Ensure confidence score is between 0.0 and 1.0."""
        if v is not None and not (0.0 <= v <= 1.0):
            raise ValueError("Confidence score must be between 0.0 and 1.0")
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "response_id": "770e8400-e29b-41d4-a716-446655440002",
                "query_id": "660e8400-e29b-41d4-a716-446655440001",
                "response_text": "A robotic control system consists of sensors (e.g., cameras, LiDAR), actuators (motors, servos)...",
                "confidence_score": 0.87,
                "model_used": "openai/gpt-4-turbo-preview",
                "timestamp": "2025-12-17T10:05:02Z",
                "token_count": 145,
            }
        }


class ResponseCreate(BaseModel):
    """Schema for creating a new response record."""

    query_id: UUID
    response_text: str = Field(..., min_length=50, max_length=2000)
    confidence_score: Optional[float] = Field(None, ge=0.0, le=1.0)
    model_used: str = Field(..., max_length=100)
    token_count: Optional[int] = Field(None, ge=0)


# Import SourceCitation here to avoid circular import
from .citation import SourceCitation


class QueryResponse(BaseModel):
    """
    API response schema for query endpoint.
    Based on OpenAPI specification.
    """

    query_id: UUID = Field(..., description="Unique identifier for this query")
    response_text: str = Field(..., description="The chatbot's answer")
    citations: List[SourceCitation] = Field(..., min_length=1, description="Source citations")
    mode: str = Field(..., description="Query mode used")
    processing_time_ms: int = Field(..., ge=0, description="Total processing time")
    confidence_score: Optional[float] = Field(
        None, ge=0.0, le=1.0, description="Confidence score from LLM"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "query_id": "660e8400-e29b-41d4-a716-446655440001",
                "response_text": "A robotic control system consists of sensors, actuators, a controller, and a planning module.",
                "citations": [
                    {
                        "chapter_name": "Chapter 1: Introduction to ROS 2",
                        "section_name": "1.2 Core Concepts",
                        "relevance_score": 0.92,
                        "link": "/docs/module-01/lesson-01#ros2-core-concepts",
                    }
                ],
                "mode": "full_book",
                "processing_time_ms": 2340,
                "confidence_score": 0.87,
            }
        }
