"""
Logging infrastructure for the OpenAI RAG Agent feature.

This module contains logging utilities and configuration for agent operations,
providing structured logging with appropriate levels and formatting for
debugging and monitoring agent behavior.
"""

import logging
import sys
from typing import Optional
from datetime import datetime
import json
import traceback
from enum import Enum


class AgentLogLevel(str, Enum):
    """Enumeration of agent-specific log levels."""

    AGENT_DEBUG = "AGENT_DEBUG"
    AGENT_INFO = "AGENT_INFO"
    AGENT_WARN = "AGENT_WARN"
    AGENT_ERROR = "AGENT_ERROR"
    AGENT_CRITICAL = "AGENT_CRITICAL"


class AgentLogFormatter(logging.Formatter):
    """Custom formatter for agent logs with structured output."""

    def format(self, record):
        """Format the log record with agent-specific structure."""
        log_entry = {
            "timestamp": datetime.fromtimestamp(record.created).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }

        # Add extra fields if present
        if hasattr(record, 'agent_operation'):
            log_entry['agent_operation'] = record.agent_operation
        if hasattr(record, 'query_id'):
            log_entry['query_id'] = record.query_id
        if hasattr(record, 'thread_id'):
            log_entry['thread_id'] = record.thread_id
        if hasattr(record, 'agent_id'):
            log_entry['agent_id'] = record.agent_id
        if hasattr(record, 'response_time'):
            log_entry['response_time'] = record.response_time
        if hasattr(record, 'tokens_used'):
            log_entry['tokens_used'] = record.tokens_used
        if hasattr(record, 'retrieved_chunks'):
            log_entry['retrieved_chunks'] = record.retrieved_chunks

        # Add exception info if present
        if record.exc_info:
            log_entry['exception'] = {
                "type": record.exc_info[0].__name__,
                "message": str(record.exc_info[1]),
                "traceback": traceback.format_exception(*record.exc_info)
            }

        return json.dumps(log_entry, ensure_ascii=False)


def setup_agent_logging(
    level: str = "INFO",
    log_file: Optional[str] = None,
    enable_console: bool = True
) -> logging.Logger:
    """
    Set up the agent logging infrastructure with appropriate configuration.

    Args:
        level: The logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Optional file path to write logs to
        enable_console: Whether to also log to console

    Returns:
        Logger: Configured logger instance for agent operations
    """
    # Create logger for agent operations
    logger = logging.getLogger("rag_agent")
    logger.setLevel(getattr(logging, level.upper()))

    # Clear existing handlers to avoid duplicates
    logger.handlers.clear()

    # Create formatter
    formatter = AgentLogFormatter()

    # Add console handler if enabled
    if enable_console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    # Add file handler if specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    # Set propagation to False to avoid duplicate logs
    logger.propagate = False

    # Log initialization
    logger.info("Agent logging infrastructure initialized", extra={
        "agent_operation": "logging_init",
        "config": {
            "level": level,
            "log_file": log_file,
            "console_enabled": enable_console
        }
    })

    return logger


def get_agent_logger(name: str) -> logging.Logger:
    """
    Get a logger instance for agent operations with consistent configuration.

    Args:
        name: Name of the logger (typically __name__ of the calling module)

    Returns:
        Logger: Configured logger instance
    """
    # Create a child logger with the provided name
    parent_logger = logging.getLogger("rag_agent")
    if not parent_logger.handlers:
        # If the parent logger isn't configured, set it up with defaults
        setup_agent_logging()

    logger = parent_logger.getChild(name)
    return logger


def log_agent_query_start(
    logger: logging.Logger,
    query: str,
    query_id: str,
    thread_id: Optional[str] = None
) -> None:
    """
    Log the start of an agent query operation.

    Args:
        logger: The logger instance to use
        query: The query being processed
        query_id: Unique identifier for the query
        thread_id: Optional thread ID for the operation
    """
    extra = {
        "agent_operation": "query_start",
        "query_id": query_id,
        "query_preview": query[:100] if len(query) > 100 else query
    }
    if thread_id:
        extra["thread_id"] = thread_id

    logger.info(f"Starting agent query processing", extra=extra)


def log_agent_query_end(
    logger: logging.Logger,
    query_id: str,
    response_length: int,
    response_time: float,
    tokens_used: Optional[int] = None,
    retrieved_chunks: Optional[int] = None,
    thread_id: Optional[str] = None
) -> None:
    """
    Log the end of an agent query operation.

    Args:
        logger: The logger instance to use
        query_id: Unique identifier for the query
        response_length: Length of the response in characters
        response_time: Time taken to process the query in seconds
        tokens_used: Number of tokens used in the response (optional)
        retrieved_chunks: Number of chunks retrieved (optional)
        thread_id: Optional thread ID for the operation
    """
    extra = {
        "agent_operation": "query_end",
        "query_id": query_id,
        "response_length": response_length,
        "response_time": response_time
    }
    if tokens_used is not None:
        extra["tokens_used"] = tokens_used
    if retrieved_chunks is not None:
        extra["retrieved_chunks"] = retrieved_chunks
    if thread_id:
        extra["thread_id"] = thread_id

    logger.info(
        f"Agent query completed - Response length: {response_length}, "
        f"Time: {response_time:.2f}s",
        extra=extra
    )


def log_agent_retrieval(
    logger: logging.Logger,
    query: str,
    retrieved_chunks: int,
    retrieval_time: float,
    query_id: Optional[str] = None
) -> None:
    """
    Log a retrieval operation by the agent.

    Args:
        logger: The logger instance to use
        query: The query used for retrieval
        retrieved_chunks: Number of chunks retrieved
        retrieval_time: Time taken for retrieval in seconds
        query_id: Optional query ID for correlation
    """
    extra = {
        "agent_operation": "retrieval",
        "retrieved_chunks": retrieved_chunks,
        "retrieval_time": retrieval_time,
        "query_preview": query[:100] if len(query) > 100 else query
    }
    if query_id:
        extra["query_id"] = query_id

    logger.info(
        f"Agent retrieval completed - Chunks: {retrieved_chunks}, "
        f"Time: {retrieval_time:.2f}s",
        extra=extra
    )


def log_agent_error(
    logger: logging.Logger,
    error: Exception,
    operation: str,
    query_id: Optional[str] = None,
    thread_id: Optional[str] = None
) -> None:
    """
    Log an error that occurred during agent operations.

    Args:
        logger: The logger instance to use
        error: The error that occurred
        operation: The operation during which the error occurred
        query_id: Optional query ID for correlation
        thread_id: Optional thread ID for correlation
    """
    extra = {
        "agent_operation": "error",
        "operation": operation,
        "error_type": type(error).__name__
    }
    if query_id:
        extra["query_id"] = query_id
    if thread_id:
        extra["thread_id"] = thread_id

    logger.error(
        f"Agent {operation} failed: {str(error)}",
        extra=extra,
        exc_info=True
    )


def log_agent_validation(
    logger: logging.Logger,
    query: str,
    grounding_score: float,
    validation_result: dict,
    query_id: Optional[str] = None
) -> None:
    """
    Log the validation of an agent response.

    Args:
        logger: The logger instance to use
        query: The original query
        grounding_score: The grounding score of the response
        validation_result: The full validation result
        query_id: Optional query ID for correlation
    """
    extra = {
        "agent_operation": "validation",
        "grounding_score": grounding_score,
        "validation_result": validation_result,
        "query_preview": query[:100] if len(query) > 100 else query
    }
    if query_id:
        extra["query_id"] = query_id

    logger.info(
        f"Agent response validation - Grounding score: {grounding_score:.2f}",
        extra=extra
    )


def log_agent_tool_call(
    logger: logging.Logger,
    tool_name: str,
    tool_input: dict,
    result_length: int,
    execution_time: float,
    query_id: Optional[str] = None
) -> None:
    """
    Log a tool call made by the agent.

    Args:
        logger: The logger instance to use
        tool_name: Name of the tool called
        tool_input: Input parameters for the tool
        result_length: Length of the tool result
        execution_time: Time taken to execute the tool
        query_id: Optional query ID for correlation
    """
    extra = {
        "agent_operation": "tool_call",
        "tool_name": tool_name,
        "result_length": result_length,
        "execution_time": execution_time
    }
    if query_id:
        extra["query_id"] = query_id

    logger.info(
        f"Agent tool call - Tool: {tool_name}, "
        f"Result length: {result_length}, Time: {execution_time:.2f}s",
        extra=extra
    )


def log_agent_thread_operation(
    logger: logging.Logger,
    operation: str,
    thread_id: str,
    query_id: Optional[str] = None
) -> None:
    """
    Log a thread-related operation by the agent.

    Args:
        logger: The logger instance to use
        operation: The thread operation (create, run, delete, etc.)
        thread_id: The thread ID
        query_id: Optional query ID for correlation
    """
    extra = {
        "agent_operation": "thread_operation",
        "thread_operation": operation,
        "thread_id": thread_id
    }
    if query_id:
        extra["query_id"] = query_id

    logger.info(f"Agent thread {operation} - Thread ID: {thread_id}", extra=extra)


# Global logger instance for easy access
_agent_logger: Optional[logging.Logger] = None


def get_global_agent_logger() -> logging.Logger:
    """
    Get the global agent logger instance.

    Returns:
        Logger: Global agent logger instance
    """
    global _agent_logger
    if _agent_logger is None:
        _agent_logger = setup_agent_logging()
    return _agent_logger


# Initialize the global logger when this module is imported
if _agent_logger is None:
    _agent_logger = setup_agent_logging(level="INFO")