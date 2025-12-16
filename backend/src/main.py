"""
FastAPI main application for RAG Chatbot backend.
Handles startup/shutdown events and route registration.
"""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import get_settings
from .db.neon_client import NeonClient, neon_client as _neon_client
from .db.qdrant_client import QdrantVectorClient, qdrant_client as _qdrant_client

# Import routers
from .api.routes import health, index, query

# Import middleware
from .api.middleware import RequestLoggingMiddleware, RateLimitMiddleware

# Import background tasks
from .tasks.scheduler import get_task_scheduler

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Get settings
settings = get_settings()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events.
    Initializes database connections on startup and closes them on shutdown.
    """
    # Startup: Initialize database connections
    logger.info("Starting RAG Chatbot API...")
    logger.info(f"Environment: {settings.environment}")

    try:
        # Initialize Neon Postgres client
        global _neon_client
        _neon_client = NeonClient(
            database_url=settings.database_url,
            min_size=settings.db_pool_min_size,
            max_size=settings.db_pool_max_size,
        )
        await _neon_client.connect()

        # Run database migrations
        await _neon_client.run_migrations()

        # Initialize Qdrant client
        global _qdrant_client
        _qdrant_client = QdrantVectorClient(
            qdrant_url=settings.qdrant_url,
            qdrant_api_key=settings.qdrant_api_key,
        )
        await _qdrant_client.connect()

        logger.info("Database connections initialized successfully")

        # Start background task scheduler
        task_scheduler = get_task_scheduler()
        await task_scheduler.start()
        logger.info("Background task scheduler started")

    except Exception as e:
        logger.error(f"Failed to initialize database connections: {e}")
        raise

    yield

    # Shutdown: Close database connections and stop background tasks
    logger.info("Shutting down RAG Chatbot API...")
    try:
        # Stop background scheduler
        task_scheduler = get_task_scheduler()
        await task_scheduler.stop()
        logger.info("Background task scheduler stopped")

        # Close database connections
        if _neon_client:
            await _neon_client.disconnect()
        if _qdrant_client:
            await _qdrant_client.disconnect()
        logger.info("Database connections closed successfully")
    except Exception as e:
        logger.error(f"Error during shutdown: {e}")


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Backend API for Integrated RAG Chatbot embedded in Physical AI & Humanoid Robotics book",
    version="1.0.0",
    lifespan=lifespan,
)

# Configure middleware (order matters: last added = first executed)
# 1. CORS (must be first to handle preflight requests)
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["*"],
)

# 2. Request logging
app.add_middleware(RequestLoggingMiddleware)

# 3. Rate limiting (20 requests per minute per session)
app.add_middleware(RateLimitMiddleware, max_requests=20, window_seconds=60)

# Register routers
app.include_router(health.router, prefix="/v1", tags=["Health"])
app.include_router(index.router, prefix="/v1/admin", tags=["Admin"])
app.include_router(query.router, prefix="/v1", tags=["Chat"])

# Root endpoint
@app.get("/")
async def root():
    """Root endpoint - API information."""
    return {
        "name": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "running",
        "environment": settings.environment,
        "docs_url": "/docs",
    }


# Make global clients accessible (set during lifespan)
__all__ = ["app", "_neon_client", "_qdrant_client"]
