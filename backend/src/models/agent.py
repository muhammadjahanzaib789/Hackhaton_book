"""
Agent models for the OpenAI RAG Agent feature.

This module contains Pydantic data models for agent requests, responses,
and related data structures used in the RAG system.
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime


class AgentRequest(BaseModel):
    """
    Model for requests to the RAG Agent.

    Attributes:
        query: The question or query to be answered by the agent
        max_tokens: Maximum number of tokens in the response (optional)
        temperature: Temperature setting for response generation (optional)
        metadata: Additional metadata for the request (optional)
    """
    query: str = Field(..., min_length=1, max_length=2000, description="The question or query to be answered by the agent")
    max_tokens: Optional[int] = Field(None, ge=1, le=4096, description="Maximum number of tokens in the response")
    temperature: Optional[float] = Field(0.7, ge=0.0, le=2.0, description="Temperature setting for response generation")
    metadata: Optional[Dict[str, Any]] = Field(None, max_length=100, description="Additional metadata for the request")


class SourceCitation(BaseModel):
    """
    Model for source citations in agent responses.

    Attributes:
        id: Unique identifier for the citation
        source_document: Name or identifier of the source document
        document_title: Title of the source document (optional)
        page_number: Page number where the information appears (optional)
        section_title: Title of the section where the information appears (optional)
        chunk_text: Text of the retrieved chunk
        relevance_score: Score indicating relevance of the citation (0.0 to 1.0)
        url: URL linking to the original source (optional)
        position_in_document: Position of the chunk in the original document (optional)
        context_before: Text before the cited chunk for context (optional)
        context_after: Text after the cited chunk for context (optional)
        citation_format: Format of the citation (e.g., APA, MLA, Chicago) (optional)
        access_date: Date when the source was accessed (optional)
        retrieved_at: Timestamp when the citation was retrieved
    """
    id: str = Field(..., description="Unique identifier for the citation")
    source_document: str = Field(..., description="Name or identifier of the source document")
    document_title: Optional[str] = Field(None, description="Title of the source document")
    page_number: Optional[int] = Field(None, description="Page number where the information appears")
    section_title: Optional[str] = Field(None, description="Title of the section where the information appears")
    chunk_text: str = Field(..., description="Text of the retrieved chunk")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Score indicating relevance of the citation")
    url: Optional[str] = Field(None, description="URL linking to the original source")
    position_in_document: Optional[int] = Field(None, description="Position of the chunk in the original document")
    context_before: Optional[str] = Field(None, description="Text before the cited chunk for context")
    context_after: Optional[str] = Field(None, description="Text after the cited chunk for context")
    citation_format: Optional[str] = Field(None, description="Format of the citation (e.g., APA, MLA, Chicago)")
    access_date: Optional[str] = Field(None, description="Date when the source was accessed")
    retrieved_at: datetime = Field(default_factory=datetime.now, description="Timestamp when the citation was retrieved")


class AgentResponse(BaseModel):
    """
    Model for responses from the RAG Agent.

    Attributes:
        answer: The agent's answer to the query
        citations: List of source citations supporting the answer
        query: Echo of the original query
        timestamp: Timestamp of the response
        tokens_used: Number of tokens used in the response (optional)
        confidence: Confidence score of the response (0.0 to 1.0) (optional)
    """
    answer: str = Field(..., description="The agent's answer to the query")
    citations: List[SourceCitation] = Field(default_factory=list, description="List of source citations supporting the answer")
    query: str = Field(..., description="Echo of the original query")
    timestamp: datetime = Field(default_factory=datetime.now, description="Timestamp of the response")
    tokens_used: Optional[int] = Field(None, description="Number of tokens used in the response")
    confidence: Optional[float] = Field(None, ge=0.0, le=1.0, description="Confidence score of the response")


class RetrievalToolRequest(BaseModel):
    """
    Model for requests to the retrieval tool used by the agent.

    Attributes:
        query: Query text for the retrieval tool
        top_k: Number of top results to retrieve (default 5)
        min_relevance_score: Minimum relevance score for results (default 0.5)
    """
    query: str = Field(..., description="Query text for the retrieval tool")
    top_k: int = Field(5, description="Number of top results to retrieve")
    min_relevance_score: float = Field(0.5, ge=0.0, le=1.0, description="Minimum relevance score for results")


class RetrievalToolResponse(BaseModel):
    """
    Model for responses from the retrieval tool used by the agent.

    Attributes:
        retrieved_chunks: List of retrieved text chunks with metadata
        query: Echo of the original query
        timestamp: Timestamp of the response
    """
    retrieved_chunks: List[Dict[str, Any]] = Field(..., description="List of retrieved text chunks with metadata")
    query: str = Field(..., description="Echo of the original query")
    timestamp: datetime = Field(default_factory=datetime.now, description="Timestamp of the response")


class RetrievedChunk(BaseModel):
    """
    Model for individual retrieved chunks from the retrieval pipeline.

    Attributes:
        id: Unique identifier for the chunk
        content: Text content of the chunk
        source_document: Name or identifier of the source document
        page_number: Page number where the chunk appears (optional)
        relevance_score: Relevance score of the chunk (0.0 to 1.0)
        metadata: Additional metadata associated with the chunk (optional)
    """
    id: str = Field(..., description="Unique identifier for the chunk")
    content: str = Field(..., description="Text content of the chunk")
    source_document: str = Field(..., description="Name or identifier of the source document")
    page_number: Optional[int] = Field(None, description="Page number where the chunk appears")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score of the chunk")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata associated with the chunk")


class ValidateResponseRequest(BaseModel):
    """
    Model for validating agent responses.

    Attributes:
        query: The original query that generated the response
        response: The agent's response to validate
        citations: Optional list of citations to validate against
    """
    query: str = Field(..., min_length=1, max_length=2000, description="The original query that generated the response")
    response: str = Field(..., min_length=1, max_length=10000, description="The agent's response to validate")
    citations: Optional[List[Dict[str, Any]]] = Field(None, description="Optional list of citations to validate against")


class FeedbackRequest(BaseModel):
    """
    Model for submitting feedback on agent responses.

    Attributes:
        query: The original query
        response: The agent's response
        feedback: User's feedback on the response
        rating: Optional numerical rating (1-5)
    """
    query: str = Field(..., min_length=1, max_length=2000, description="The original query")
    response: str = Field(..., min_length=1, max_length=10000, description="The agent's response")
    feedback: str = Field(..., min_length=1, max_length=5000, description="User's feedback on the response")
    rating: Optional[int] = Field(None, ge=1, le=5, description="Optional numerical rating (1-5)")


class ErrorResponse(BaseModel):
    """
    Model for error responses from the agent system.

    Attributes:
        error_code: Standardized error code
        message: Human-readable error message
        details: Detailed error information (optional)
        timestamp: Timestamp of the error
    """
    error_code: str = Field(..., description="Standardized error code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[Dict[str, Any]] = Field(None, description="Detailed error information")
    timestamp: datetime = Field(default_factory=datetime.now, description="Timestamp of the error")