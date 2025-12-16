"""
Models package for RAG Chatbot.
Exports all Pydantic models for easy import.
"""

from .chunk import BookContentChunk, BookContentChunkCreate
from .session import ChatSession, ChatSessionCreate, ChatSessionUpdate, SessionMode, SessionStatus
from .query import Query, QueryRequest, QueryCreate
from .response import Response, ResponseCreate, QueryResponse
from .citation import SourceCitation, SourceCitationCreate

__all__ = [
    # Chunk models
    "BookContentChunk",
    "BookContentChunkCreate",
    # Session models
    "ChatSession",
    "ChatSessionCreate",
    "ChatSessionUpdate",
    "SessionMode",
    "SessionStatus",
    # Query models
    "Query",
    "QueryRequest",
    "QueryCreate",
    # Response models
    "Response",
    "ResponseCreate",
    "QueryResponse",
    # Citation models
    "SourceCitation",
    "SourceCitationCreate",
]
