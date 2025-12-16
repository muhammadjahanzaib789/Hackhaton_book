"""
Middleware for FastAPI application.
Currently CORS is handled in main.py, but this file can be extended for custom middleware.
"""

import logging
import time
from typing import Callable

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

logger = logging.getLogger(__name__)


class RequestLoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware to log all HTTP requests and responses.
    Useful for debugging and monitoring.
    """

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Log request details and processing time."""
        start_time = time.time()

        # Log request
        logger.info(f"Request: {request.method} {request.url.path}")

        # Process request
        response = await call_next(request)

        # Calculate processing time
        process_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        response.headers["X-Process-Time"] = f"{process_time:.2f}ms"

        # Log response
        logger.info(
            f"Response: {response.status_code} {request.url.path} - {process_time:.2f}ms"
        )

        return response


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Simple rate limiting middleware.
    Note: For production, consider using Redis-based rate limiting.
    """

    def __init__(self, app, max_requests: int = 20, window_seconds: int = 60):
        super().__init__(app)
        self.max_requests = max_requests
        self.window_seconds = window_seconds
        self.request_counts = {}  # In-memory storage (session_id -> [timestamps])

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Check rate limit per session_id."""
        # Extract session_id from request (if available)
        session_id = request.headers.get("X-Session-ID") or request.client.host

        # Clean old timestamps
        current_time = time.time()
        if session_id in self.request_counts:
            self.request_counts[session_id] = [
                ts for ts in self.request_counts[session_id]
                if current_time - ts < self.window_seconds
            ]

        # Check rate limit
        if session_id in self.request_counts:
            if len(self.request_counts[session_id]) >= self.max_requests:
                logger.warning(f"Rate limit exceeded for session: {session_id}")
                return Response(
                    content='{"error": "RATE_LIMIT_EXCEEDED", "message": "Too many requests. Please try again later."}',
                    status_code=429,
                    media_type="application/json",
                )

        # Record request
        if session_id not in self.request_counts:
            self.request_counts[session_id] = []
        self.request_counts[session_id].append(current_time)

        # Process request
        response = await call_next(request)
        return response


# CORS configuration is handled in main.py using CORSMiddleware
# This ensures proper handling of preflight requests and cross-origin policies
