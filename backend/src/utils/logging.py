"""
Logging infrastructure for the RAG Retrieval Validation feature.

This module sets up structured logging for retrieval operations
with performance metrics and debugging information.
"""

import asyncio
import logging
import time
from typing import Callable, Any
from functools import wraps
from datetime import datetime


def setup_logging():
    """Set up logging configuration for the retrieval services."""
    # Create logger for retrieval operations
    retrieval_logger = logging.getLogger('retrieval')
    retrieval_logger.setLevel(logging.INFO)
    
    # Create console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)
    
    # Add handler to logger
    if not retrieval_logger.handlers:
        retrieval_logger.addHandler(console_handler)
    
    return retrieval_logger


def log_execution_time(func: Callable) -> Callable:
    """Decorator to log execution time of functions."""
    @wraps(func)
    async def async_wrapper(*args, **kwargs):
        start_time = time.time()
        try:
            result = await func(*args, **kwargs)
            execution_time = time.time() - start_time
            logging.getLogger('retrieval').info(
                f"{func.__name__} executed in {execution_time:.4f} seconds"
            )
            return result
        except Exception as e:
            execution_time = time.time() - start_time
            logging.getLogger('retrieval').error(
                f"{func.__name__} failed after {execution_time:.4f} seconds: {str(e)}"
            )
            raise
    
    @wraps(func)
    def sync_wrapper(*args, **kwargs):
        start_time = time.time()
        try:
            result = func(*args, **kwargs)
            execution_time = time.time() - start_time
            logging.getLogger('retrieval').info(
                f"{func.__name__} executed in {execution_time:.4f} seconds"
            )
            return result
        except Exception as e:
            execution_time = time.time() - start_time
            logging.getLogger('retrieval').error(
                f"{func.__name__} failed after {execution_time:.4f} seconds: {str(e)}"
            )
            raise
    
    # Determine if the function is async or sync
    if asyncio.iscoroutinefunction(func):
        return async_wrapper
    else:
        return sync_wrapper


def log_retrieval_request(query_text: str, top_k: int, filters: dict = None):
    """Log retrieval request details."""
    logger = logging.getLogger('retrieval')
    filters_str = str(filters) if filters else "None"
    logger.info(f"Retrieval request - Query: '{query_text[:50]}...', top_k: {top_k}, filters: {filters_str}")


def log_retrieval_response(query_id: str, result_count: int, query_latency: float):
    """Log retrieval response details."""
    logger = logging.getLogger('retrieval')
    logger.info(f"Retrieval response - Query ID: {query_id}, Results: {result_count}, Latency: {query_latency:.4f}s")


def log_validation_request(benchmark_name: str, question_count: int):
    """Log validation request details."""
    logger = logging.getLogger('retrieval')
    logger.info(f"Validation request - Benchmark: {benchmark_name}, Questions: {question_count}")


def log_validation_response(benchmark_name: str, metrics: dict, validation_latency: float):
    """Log validation response details."""
    logger = logging.getLogger('retrieval')
    logger.info(f"Validation response - Benchmark: {benchmark_name}, Metrics: {metrics}, Latency: {validation_latency:.4f}s")


def get_retrieval_timer():
    """Context manager for timing retrieval operations."""
    class Timer:
        def __enter__(self):
            self.start = time.time()
            return self

        def __exit__(self, *args):
            self.end = time.time()
            self.interval = self.end - self.start
    
    return Timer()


# Initialize the main logger
main_logger = setup_logging()