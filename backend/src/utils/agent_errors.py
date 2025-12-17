"""
Error handling utilities for the OpenAI RAG Agent feature.

This module contains custom exceptions and error handling utilities
for agent operations, providing structured error responses and
proper error categorization.
"""

from typing import Optional, Dict, Any
from enum import Enum
import logging
from datetime import datetime


class AgentErrorCode(str, Enum):
    """Enumeration of agent-specific error codes."""

    # Configuration errors
    AGENT_CONFIG_ERROR = "AGENT_CONFIG_ERROR"
    AGENT_MISSING_API_KEY = "AGENT_MISSING_API_KEY"
    AGENT_INVALID_MODEL = "AGENT_INVALID_MODEL"

    # Runtime errors
    AGENT_RUNTIME_ERROR = "AGENT_RUNTIME_ERROR"
    AGENT_TIMEOUT_ERROR = "AGENT_TIMEOUT_ERROR"
    AGENT_CONNECTION_ERROR = "AGENT_CONNECTION_ERROR"

    # Retrieval errors
    AGENT_RETRIEVAL_ERROR = "AGENT_RETRIEVAL_ERROR"
    AGENT_RETRIEVAL_TIMEOUT = "AGENT_RETRIEVAL_TIMEOUT"
    AGENT_RETRIEVAL_EMPTY = "AGENT_RETRIEVAL_EMPTY"

    # Validation errors
    AGENT_VALIDATION_ERROR = "AGENT_VALIDATION_ERROR"
    AGENT_GROUNDING_ERROR = "AGENT_GROUNDING_ERROR"
    AGENT_CITATION_ERROR = "AGENT_CITATION_ERROR"

    # OpenAI API errors
    OPENAI_API_ERROR = "OPENAI_API_ERROR"
    OPENAI_RATE_LIMIT_ERROR = "OPENAI_RATE_LIMIT_ERROR"
    OPENAI_AUTHENTICATION_ERROR = "OPENAI_AUTHENTICATION_ERROR"
    OPENAI_SERVICE_ERROR = "OPENAI_SERVICE_ERROR"


class AgentError(Exception):
    """Base exception class for agent-related errors."""

    def __init__(
        self,
        message: str,
        error_code: AgentErrorCode,
        details: Optional[Dict[str, Any]] = None,
        original_exception: Optional[Exception] = None
    ):
        super().__init__(message)
        self.message = message
        self.error_code = error_code
        self.details = details or {}
        self.original_exception = original_exception
        self.timestamp = datetime.now()

    def to_dict(self) -> Dict[str, Any]:
        """Convert the error to a dictionary representation."""
        result = {
            "error_code": self.error_code.value,
            "message": self.message,
            "timestamp": self.timestamp.isoformat(),
            "details": self.details
        }

        if self.original_exception:
            result["original_error"] = str(self.original_exception)

        return result


class AgentConfigurationError(AgentError):
    """Exception raised for agent configuration errors."""

    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            message=message,
            error_code=AgentErrorCode.AGENT_CONFIG_ERROR,
            details=details
        )


class AgentRetrievalError(AgentError):
    """Exception raised for agent retrieval errors."""

    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            message=message,
            error_code=AgentErrorCode.AGENT_RETRIEVAL_ERROR,
            details=details
        )


class AgentValidationError(AgentError):
    """Exception raised for agent validation errors."""

    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            message=message,
            error_code=AgentErrorCode.AGENT_VALIDATION_ERROR,
            details=details
        )


class AgentTimeoutError(AgentError):
    """Exception raised when agent operations timeout."""

    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            message=message,
            error_code=AgentErrorCode.AGENT_TIMEOUT_ERROR,
            details=details
        )


class AgentGroundingError(AgentError):
    """Exception raised when agent responses are not properly grounded."""

    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            message=message,
            error_code=AgentErrorCode.AGENT_GROUNDING_ERROR,
            details=details
        )


def handle_agent_error(
    exception: Exception,
    error_code: AgentErrorCode,
    logger: Optional[logging.Logger] = None,
    context: Optional[Dict[str, Any]] = None
) -> AgentError:
    """
    Handle an exception and convert it to an appropriate AgentError.

    Args:
        exception: The original exception to handle
        error_code: The error code to use for the AgentError
        logger: Optional logger for logging the error
        context: Optional context information to include in details

    Returns:
        AgentError: The converted agent error
    """
    if isinstance(exception, AgentError):
        # If it's already an AgentError, return it as is
        return exception

    # Create error details
    details = context or {}
    details["exception_type"] = type(exception).__name__

    # Log the error if logger is provided
    if logger:
        logger.error(
            f"Agent error occurred: {str(exception)}",
            extra={"error_code": error_code.value, "context": context},
            exc_info=True
        )

    # Create and return the appropriate AgentError
    return AgentError(
        message=str(exception),
        error_code=error_code,
        details=details,
        original_exception=exception
    )


def validate_agent_response_grounding(response: str, retrieved_chunks: list) -> bool:
    """
    Validate that a response is properly grounded in retrieved content.

    Args:
        response: The agent's response to validate
        retrieved_chunks: List of chunks retrieved for the query

    Returns:
        bool: True if the response is properly grounded, False otherwise
    """
    if not retrieved_chunks:
        return False

    # Simple validation: check if response content overlaps with retrieved chunks
    response_lower = response.lower()
    retrieved_content = " ".join([chunk.get('content', '') if isinstance(chunk, dict) else str(chunk) for chunk in retrieved_chunks]).lower()

    # Check for content overlap
    response_words = set(response_lower.split())
    retrieved_words = set(retrieved_content.split())
    overlap = response_words.intersection(retrieved_words)

    # Consider it grounded if there's at least 30% overlap
    if len(response_words) == 0:
        return len(retrieved_words) == 0

    overlap_ratio = len(overlap) / len(response_words)
    return overlap_ratio >= 0.3


def create_error_response(
    error_code: AgentErrorCode,
    message: str,
    details: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Create a standardized error response dictionary.

    Args:
        error_code: The error code
        message: The error message
        details: Optional additional details

    Returns:
        Dict: Standardized error response
    """
    response = {
        "error": {
            "code": error_code.value,
            "message": message,
            "timestamp": datetime.now().isoformat()
        }
    }

    if details:
        response["error"]["details"] = details

    return response


def log_agent_error(
    logger: logging.Logger,
    error: AgentError,
    operation: str = "unknown"
) -> None:
    """
    Log an agent error with appropriate details.

    Args:
        logger: The logger to use
        error: The agent error to log
        operation: The operation during which the error occurred
    """
    logger.error(
        f"Agent {operation} failed: {error.message}",
        extra={
            "error_code": error.error_code.value,
            "details": error.details,
            "timestamp": error.timestamp.isoformat()
        },
        exc_info=error.original_exception
    )


# Backward compatibility for existing code
class OpenAIError(Exception):
    """Deprecated: Use AgentError instead."""

    def __init__(self, message: str):
        super().__init__(message)
        import warnings
        warnings.warn(
            "OpenAIError is deprecated, use AgentError instead",
            DeprecationWarning,
            stacklevel=2
        )