"""
BookContentChunk model for book content segments.
Based on data-model.md specification.
"""

from datetime import datetime
from typing import Optional, List
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator


class BookContentChunk(BaseModel):
    """
    Represents a segment of book content with metadata and embedding.

    Storage: Qdrant (vector) + Neon Postgres (metadata)
    """

    chunk_id: UUID = Field(default_factory=uuid4, description="Unique identifier for the chunk")
    text: str = Field(..., min_length=100, max_length=2000, description="The actual text content")
    embedding_vector: Optional[List[float]] = Field(
        None, description="Vector embedding (768 dimensions)"
    )
    chapter_name: str = Field(..., max_length=200, description="Chapter title")
    section_name: Optional[str] = Field(None, max_length=200, description="Section or subsection title")
    page_number: Optional[int] = Field(None, ge=1, description="Page number in the book")
    document_path: str = Field(..., max_length=500, description="Path to source Markdown file")
    chunk_index: int = Field(..., ge=0, description="Sequential index within the document")
    token_count: int = Field(..., ge=100, le=500, description="Number of tokens in the chunk")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When chunk was indexed")
    metadata: Optional[dict] = Field(None, description="Additional metadata (tags, code_block, etc.)")

    @field_validator("embedding_vector")
    @classmethod
    def validate_embedding_dimensions(cls, v):
        """Validate embedding vector has exactly 768 dimensions."""
        if v is not None and len(v) != 768:
            raise ValueError("Embedding vector must have exactly 768 dimensions")
        return v

    @field_validator("text")
    @classmethod
    def validate_text_length(cls, v):
        """Enforce text length between 100 and 2000 characters."""
        if not (100 <= len(v) <= 2000):
            raise ValueError("Text length must be between 100 and 2000 characters")
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "123e4567-e89b-12d3-a456-426614174000",
                "text": "ROS 2 (Robot Operating System 2) is a set of software libraries and tools...",
                "chapter_name": "Chapter 1: Introduction to ROS 2",
                "section_name": "1.1 What is ROS 2?",
                "page_number": 5,
                "document_path": "docs/module-01/lesson-01.md",
                "chunk_index": 0,
                "token_count": 150,
                "metadata": {"code_block": False, "topic": "ros2-intro"},
            }
        }


class BookContentChunkCreate(BaseModel):
    """Schema for creating a new book content chunk."""

    text: str = Field(..., min_length=100, max_length=2000)
    embedding_vector: List[float] = Field(..., description="768-dimensional vector")
    chapter_name: str = Field(..., max_length=200)
    section_name: Optional[str] = Field(None, max_length=200)
    page_number: Optional[int] = Field(None, ge=1)
    document_path: str = Field(..., max_length=500)
    chunk_index: int = Field(..., ge=0)
    token_count: int = Field(..., ge=100, le=500)
    metadata: Optional[dict] = None
