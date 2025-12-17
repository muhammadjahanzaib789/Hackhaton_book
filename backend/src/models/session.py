"""
ChatSession model for user interaction sessions.
Based on data-model.md specification.
"""

from datetime import datetime
from enum import Enum
from typing import Optional
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator


class SessionMode(str, Enum):
    """Query mode enum."""

    FULL_BOOK = "full_book"
    SELECTED_TEXT = "selected_text"


class SessionStatus(str, Enum):
    """Session status enum."""

    ACTIVE = "active"
    ENDED = "ended"
    ABANDONED = "abandoned"


class ChatSession(BaseModel):
    """
    Represents a user's interaction session with the chatbot.

    Storage: Neon Postgres
    """

    session_id: UUID = Field(default_factory=uuid4, description="Unique session identifier")
    user_id: Optional[str] = Field(None, max_length=100, description="Anonymous user identifier")
    started_at: datetime = Field(default_factory=datetime.utcnow, description="Session start time")
    last_activity_at: datetime = Field(
        default_factory=datetime.utcnow, description="Last query timestamp"
    )
    mode: SessionMode = Field(default=SessionMode.FULL_BOOK, description="Current query mode")
    query_count: int = Field(default=0, ge=0, description="Number of queries in session")
    status: SessionStatus = Field(default=SessionStatus.ACTIVE, description="Session status")

    @field_validator("last_activity_at")
    @classmethod
    def validate_activity_time(cls, v, info):
        """Ensure last_activity_at is >= started_at."""
        if "started_at" in info.data and v < info.data["started_at"]:
            raise ValueError("last_activity_at must be >= started_at")
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "user_id": "anonymous_12345",
                "started_at": "2025-12-17T10:00:00Z",
                "last_activity_at": "2025-12-17T10:05:00Z",
                "mode": "full_book",
                "query_count": 3,
                "status": "active",
            }
        }


class ChatSessionCreate(BaseModel):
    """Schema for creating a new chat session."""

    user_id: Optional[str] = Field(None, max_length=100)
    mode: SessionMode = Field(default=SessionMode.FULL_BOOK)


class ChatSessionUpdate(BaseModel):
    """Schema for updating a chat session."""

    last_activity_at: Optional[datetime] = None
    mode: Optional[SessionMode] = None
    query_count: Optional[int] = Field(None, ge=0)
    status: Optional[SessionStatus] = None
