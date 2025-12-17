"""
Error handling utilities for the RAG Retrieval Validation feature.

This module defines custom exceptions and error handling patterns
for the retrieval and validation services.
"""

from typing import Optional
from fastapi import HTTPException, status
from pydantic import BaseModel
from datetime import datetime


class RAGError(Exception):
    """Base exception class for RAG-related errors."""
    
    def __init__(self, message: str, error_code: str, details: Optional[dict] = None):
        self.message = message
        self.error_code = error_code
        self.details = details or {}
        super().__init__(self.message)


class RetrievalError(RAGError):
    """Exception for retrieval-specific errors."""
    pass


class ValidationError(RAGError):
    """Exception for validation-specific errors."""
    pass


class CohereAPIError(RAGError):
    """Exception for Cohere API-related errors."""
    pass


class QdrantAPIError(RAGError):
    """Exception for Qdrant API-related errors."""
    pass


class QueryValidationError(RAGError):
    """Exception for query validation errors."""
    pass


class ErrorResponse(BaseModel):
    """Standard error response model."""
    error: str
    error_code: str
    timestamp: datetime
    details: Optional[dict] = None


def create_error_response(error: RAGError) -> ErrorResponse:
    """Create an error response from a RAGError."""
    return ErrorResponse(
        error=error.message,
        error_code=error.error_code,
        timestamp=datetime.utcnow(),
        details=error.details
    )


def handle_retrieval_error(error: Exception, query_text: str = "") -> HTTPException:
    """Standard error handler for retrieval endpoints."""
    if isinstance(error, RAGError):
        return HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": error.message,
                "error_code": error.error_code,
                "query_text": query_text
            }
        )
    else:
        return HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error during retrieval",
                "error_code": "INTERNAL_ERROR",
                "query_text": query_text
            }
        )


def handle_validation_error(error: Exception, benchmark_name: str = "") -> HTTPException:
    """Standard error handler for validation endpoints."""
    if isinstance(error, RAGError):
        return HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": error.message,
                "error_code": error.error_code,
                "benchmark_name": benchmark_name
            }
        )
    else:
        return HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error during validation",
                "error_code": "INTERNAL_ERROR",
                "benchmark_name": benchmark_name
            }
        )