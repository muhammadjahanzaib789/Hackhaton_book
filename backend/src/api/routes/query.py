"""
Query endpoint for the RAG chatbot.
Orchestrates the full pipeline: validate → embed → retrieve → generate → log → respond.
"""

import logging
import time
from typing import Optional
from uuid import UUID

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field, field_validator

from ...models.session import SessionMode
from ...services.retrieval import get_retrieval_service
from ...services.llm_generation import get_llm_service
from ...services.chat_logger import get_chat_logger_service
from ...config import get_settings

logger = logging.getLogger(__name__)
router = APIRouter()
settings = get_settings()


class QueryRequest(BaseModel):
    """Request schema for querying the chatbot."""

    session_id: Optional[UUID] = Field(
        default=None,
        description="Session ID. If not provided, a new session will be created.",
    )
    query: str = Field(
        ...,
        description="User's question",
        min_length=5,
        max_length=500,
    )
    mode: SessionMode = Field(
        default=SessionMode.FULL_BOOK,
        description="Query mode: full_book or selected_text",
    )
    selected_text: Optional[str] = Field(
        default=None,
        description="Selected text from the book (required for selected_text mode)",
    )

    @field_validator("selected_text")
    @classmethod
    def validate_selected_text(cls, v, info):
        """Validate selected_text based on mode."""
        mode = info.data.get("mode")

        if mode == SessionMode.SELECTED_TEXT:
            if v is None or len(v) < 10:
                raise ValueError(
                    "selected_text must be at least 10 characters for selected_text mode"
                )
        elif mode == SessionMode.FULL_BOOK:
            if v is not None:
                raise ValueError("selected_text must be None for full_book mode")

        return v

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "123e4567-e89b-12d3-a456-426614174000",
                "query": "What are the main components of a humanoid robot control system?",
                "mode": "full_book",
                "selected_text": None,
            }
        }


class SourceCitation(BaseModel):
    """Source citation in the response."""

    chunk_id: str
    chapter_name: str
    section_name: Optional[str] = None
    page_number: Optional[int] = None
    relevance_score: float
    text_preview: str = Field(
        ..., description="First 200 characters of the source text"
    )
    excerpt: Optional[str] = Field(
        None, description="First 500 characters for hover preview"
    )
    link: Optional[str] = Field(
        None, description="Docusaurus route for navigation (e.g., /docs/chapter-01/lesson-01#section)"
    )


class QueryResponse(BaseModel):
    """Response schema for chatbot query."""

    session_id: UUID = Field(..., description="Session ID for follow-up queries")
    query_id: UUID = Field(..., description="Unique query identifier")
    answer: str = Field(..., description="Generated answer from the LLM")
    sources: list[SourceCitation] = Field(
        ..., description="Source citations used to generate the answer"
    )
    mode: SessionMode = Field(..., description="Query mode used")
    processing_time_ms: int = Field(
        ..., description="Total processing time in milliseconds"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "123e4567-e89b-12d3-a456-426614174000",
                "query_id": "987fcdeb-51a2-43d7-8901-123456789abc",
                "answer": "A humanoid robot control system consists of...",
                "sources": [
                    {
                        "chunk_id": "abc123...",
                        "chapter_name": "Chapter 3: Robot Control Systems",
                        "section_name": "Hierarchical Control Architecture",
                        "page_number": 45,
                        "relevance_score": 0.89,
                        "text_preview": "The control system architecture...",
                    }
                ],
                "mode": "full_book",
                "processing_time_ms": 1250,
            }
        }


@router.post("/query", response_model=QueryResponse, status_code=status.HTTP_200_OK)
async def query_chatbot(request: QueryRequest):
    """
    Query the RAG chatbot.

    Performs the full RAG pipeline:
    1. Validate input and create/get session
    2. Generate query embedding
    3. Retrieve relevant book chunks (vector search)
    4. Generate answer using LLM
    5. Log query, response, and citations
    6. Return answer with source citations

    Args:
        request: QueryRequest with query, mode, and optional session_id/selected_text

    Returns:
        QueryResponse: Answer, sources, and metadata

    Raises:
        400: Invalid request (validation errors)
        404: No relevant content found
        503: External service unavailable (Qdrant, OpenRouter)
        500: Internal server error
    """
    start_time = time.time()
    logger.info(
        f"Query request: query='{request.query[:50]}...', mode={request.mode.value}"
    )

    try:
        # Step 1: Session management
        chat_logger = get_chat_logger_service()

        if request.session_id is None:
            # Create new session
            session_id = await chat_logger.create_session(mode=request.mode)
            logger.info(f"Created new session {session_id}")
        else:
            # Use existing session
            session_id = request.session_id
            logger.info(f"Using existing session {session_id}")

        # Step 2: Log query
        query_id = await chat_logger.log_query(
            session_id=session_id,
            query_text=request.query,
            mode=request.mode,
            selected_text=request.selected_text,
        )

        # Step 3: Retrieve relevant chunks
        retrieval_service = get_retrieval_service()

        try:
            retrieved_chunks = await retrieval_service.retrieve(
                query=request.query,
                top_k=5,
                score_threshold=0.3,
                selected_text=request.selected_text,
                mode=request.mode.value,
            )
        except Exception as e:
            logger.error(f"Retrieval service failed: {e}", exc_info=True)
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail={
                    "error": "RETRIEVAL_SERVICE_UNAVAILABLE",
                    "message": "Vector search service is temporarily unavailable. Please try again later.",
                },
            )

        # Step 4: Check if relevant content was found
        if not retrieved_chunks:
            logger.warning("No relevant content found for query")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail={
                    "error": "NO_RELEVANT_CONTENT",
                    "message": "No relevant content found in the book for your query. Try rephrasing your question or selecting a different text passage.",
                },
            )

        # Step 5: Generate answer using LLM
        llm_service = get_llm_service()

        try:
            answer = await llm_service.generate_answer(
                query=request.query,
                retrieved_chunks=retrieved_chunks,
                mode=request.mode,
                selected_text=request.selected_text,
            )
        except TimeoutError as e:
            logger.error(f"LLM request timed out: {e}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail={
                    "error": "REQUEST_TIMEOUT",
                    "message": str(e),
                },
            )
        except Exception as e:
            logger.error(f"LLM generation failed: {e}", exc_info=True)
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail={
                    "error": "LLM_SERVICE_UNAVAILABLE",
                    "message": "Answer generation service is temporarily unavailable. Please try again later.",
                },
            )

        # Step 6: Calculate processing time
        processing_time_ms = int((time.time() - start_time) * 1000)

        # Step 7: Log response
        response_id = await chat_logger.log_response(
            query_id=query_id,
            answer_text=answer,
            model_name=settings.llm_model,
            processing_time_ms=processing_time_ms,
        )

        # Step 8: Log source citations
        await chat_logger.log_source_citations(
            response_id=response_id, retrieved_chunks=retrieved_chunks
        )

        # Step 9: Sort retrieved chunks by relevance_score DESC (T041)
        retrieved_chunks_sorted = sorted(
            retrieved_chunks, key=lambda x: x.relevance_score, reverse=True
        )

        # Step 10: Build source citations for response with excerpts and links (T039, T040)
        sources = []
        for chunk in retrieved_chunks_sorted:
            # Text preview (first 200 chars)
            text_preview = (
                chunk.text[:200] + "..." if len(chunk.text) > 200 else chunk.text
            )

            # Excerpt (first 500 chars for hover preview)
            excerpt = chunk.text[:500] if len(chunk.text) > 500 else chunk.text

            # Generate navigation link
            link = chat_logger._generate_docusaurus_link(
                chunk.document_path, chunk.section_name
            )

            sources.append(
                SourceCitation(
                    chunk_id=str(chunk.chunk_id),
                    chapter_name=chunk.chapter_name,
                    section_name=chunk.section_name,
                    page_number=chunk.page_number,
                    relevance_score=round(chunk.relevance_score, 3),
                    text_preview=text_preview,
                    excerpt=excerpt,
                    link=link,
                )
            )

        # Step 11: Build response
        response = QueryResponse(
            session_id=session_id,
            query_id=query_id,
            answer=answer,
            sources=sources,
            mode=request.mode,
            processing_time_ms=processing_time_ms,
        )

        logger.info(
            f"Query completed successfully: query_id={query_id}, processing_time={processing_time_ms}ms"
        )
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise

    except Exception as e:
        logger.error(f"Unexpected error during query processing: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "INTERNAL_SERVER_ERROR",
                "message": "An unexpected error occurred while processing your query. Please try again later.",
            },
        )
