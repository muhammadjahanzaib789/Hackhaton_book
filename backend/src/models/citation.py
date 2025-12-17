"""
SourceCitation model for book content references.
Based on data-model.md specification.
"""

from typing import Optional
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator


class SourceCitation(BaseModel):
    """
    Represents a reference to book content used in a response.

    Storage: Neon Postgres
    """

    citation_id: UUID = Field(default_factory=uuid4, description="Unique citation identifier")
    response_id: UUID = Field(..., description="Reference to response")
    chunk_id: UUID = Field(..., description="Reference to book content chunk")
    chapter_name: str = Field(..., max_length=200, description="Chapter title (denormalized)")
    section_name: Optional[str] = Field(None, max_length=200, description="Section title (denormalized)")
    page_number: Optional[int] = Field(None, ge=1, description="Page number (denormalized)")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score from vector search")
    excerpt: Optional[str] = Field(None, max_length=500, description="Short snippet of cited text")
    link: Optional[str] = Field(None, max_length=500, description="URL anchor to book location")

    @field_validator("relevance_score")
    @classmethod
    def validate_relevance_score(cls, v):
        """Ensure relevance score is between 0.0 and 1.0."""
        if not (0.0 <= v <= 1.0):
            raise ValueError("Relevance score must be between 0.0 and 1.0")
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "citation_id": "880e8400-e29b-41d4-a716-446655440003",
                "response_id": "770e8400-e29b-41d4-a716-446655440002",
                "chunk_id": "123e4567-e89b-12d3-a456-426614174000",
                "chapter_name": "Chapter 1: Introduction to ROS 2",
                "section_name": "1.2 Core Concepts",
                "page_number": 15,
                "relevance_score": 0.92,
                "excerpt": "ROS 2 uses a distributed architecture with nodes communicating via topics...",
                "link": "/docs/module-01/lesson-01#ros2-core-concepts",
            }
        }


class SourceCitationCreate(BaseModel):
    """Schema for creating a new source citation."""

    response_id: UUID
    chunk_id: UUID
    chapter_name: str = Field(..., max_length=200)
    section_name: Optional[str] = Field(None, max_length=200)
    page_number: Optional[int] = Field(None, ge=1)
    relevance_score: float = Field(..., ge=0.0, le=1.0)
    excerpt: Optional[str] = Field(None, max_length=500)
    link: Optional[str] = Field(None, max_length=500)
