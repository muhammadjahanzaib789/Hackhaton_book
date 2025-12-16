"""
Query model for user questions.
Based on data-model.md specification.
"""

from datetime import datetime
from typing import Optional
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator

from .session import SessionMode


class Query(BaseModel):
    """
    Represents a user's question submitted to the chatbot.

    Storage: Neon Postgres
    """

    query_id: UUID = Field(default_factory=uuid4, description="Unique query identifier")
    session_id: UUID = Field(..., description="Reference to chat session")
    query_text: str = Field(..., min_length=5, max_length=500, description="The user's question")
    selected_text: Optional[str] = Field(
        None, max_length=5000, description="User-selected text (for selected_text mode)"
    )
    mode: SessionMode = Field(..., description="Query mode: full_book or selected_text")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Query submission time")
    processing_time_ms: Optional[int] = Field(None, ge=0, description="Processing time in milliseconds")
    error: Optional[str] = Field(None, max_length=500, description="Error message if query failed")

    @field_validator("selected_text")
    @classmethod
    def validate_selected_text(cls, v, info):
        """
        Validate selected_text based on mode:
        - If mode=selected_text, selected_text must be >= 10 chars
        - If mode=full_book, selected_text must be None
        """
        if "mode" in info.data:
            mode = info.data["mode"]
            if mode == SessionMode.SELECTED_TEXT:
                if v is None or len(v) < 10:
                    raise ValueError("selected_text must be at least 10 characters for selected_text mode")
            elif mode == SessionMode.FULL_BOOK:
                if v is not None:
                    raise ValueError("selected_text must be None for full_book mode")
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "query_id": "660e8400-e29b-41d4-a716-446655440001",
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "query_text": "What are the key components of a robotic control system?",
                "selected_text": None,
                "mode": "full_book",
                "timestamp": "2025-12-17T10:05:00Z",
                "processing_time_ms": 2340,
                "error": None,
            }
        }


class QueryRequest(BaseModel):
    """
    API request schema for submitting a query.
    Based on OpenAPI specification.
    """

    query: str = Field(..., min_length=5, max_length=500, description="The user's question")
    session_id: UUID = Field(..., description="Unique identifier for the chat session")
    selected_text: Optional[str] = Field(
        None, min_length=10, max_length=5000, description="Text selected by the user (optional)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What are the key components of a robotic control system?",
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "selected_text": None,
            }
        }


class QueryCreate(BaseModel):
    """Schema for creating a new query record."""

    session_id: UUID
    query_text: str = Field(..., min_length=5, max_length=500)
    selected_text: Optional[str] = Field(None, max_length=5000)
    mode: SessionMode
    processing_time_ms: Optional[int] = Field(None, ge=0)
    error: Optional[str] = Field(None, max_length=500)
