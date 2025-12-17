"""
Health check endpoint for API monitoring.
Checks connectivity to Qdrant, Neon Postgres, and OpenRouter.
"""

import logging
from datetime import datetime
from typing import Literal, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from ...db.neon_client import get_neon_client
from ...db.qdrant_client import get_qdrant_client
from ...config import get_settings

logger = logging.getLogger(__name__)
router = APIRouter()
settings = get_settings()


class DependencyStatus(BaseModel):
    """Status of a single dependency."""

    status: Literal["up", "down", "degraded"]
    latency_ms: Optional[int] = None
    error: Optional[str] = None


class HealthResponse(BaseModel):
    """Health check response schema."""

    status: Literal["healthy", "degraded", "unhealthy"]
    timestamp: datetime
    dependencies: dict[str, DependencyStatus]

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "timestamp": "2025-12-17T12:34:56Z",
                "dependencies": {
                    "qdrant": {"status": "up", "latency_ms": 45, "error": None},
                    "neon_postgres": {"status": "up", "latency_ms": 23, "error": None},
                    "openrouter": {"status": "up", "latency_ms": 120, "error": None},
                },
            }
        }


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.

    Returns the health status of the API and its dependencies:
    - Qdrant Cloud (vector database)
    - Neon Serverless Postgres (metadata database)
    - OpenRouter API (LLM and embeddings)

    Returns:
        HealthResponse: Overall status and dependency details
    """
    logger.info("Health check requested")

    dependencies = {}

    # Check Neon Postgres
    try:
        neon_client = get_neon_client()
        neon_status = await neon_client.health_check()
        dependencies["neon_postgres"] = DependencyStatus(**neon_status)
    except Exception as e:
        logger.error(f"Neon Postgres health check failed: {e}")
        dependencies["neon_postgres"] = DependencyStatus(
            status="down", error=str(e)
        )

    # Check Qdrant
    try:
        qdrant_client = get_qdrant_client()
        qdrant_status = await qdrant_client.health_check()
        dependencies["qdrant"] = DependencyStatus(**qdrant_status)
    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")
        dependencies["qdrant"] = DependencyStatus(status="down", error=str(e))

    # Check OpenRouter (simple API key validation check)
    try:
        import httpx
        import time

        start_time = time.time()
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{settings.openrouter_base_url}/models",
                headers={"Authorization": f"Bearer {settings.openrouter_api_key}"},
                timeout=5.0,
            )
            latency_ms = int((time.time() - start_time) * 1000)

            if response.status_code == 200:
                dependencies["openrouter"] = DependencyStatus(
                    status="up", latency_ms=latency_ms
                )
            else:
                dependencies["openrouter"] = DependencyStatus(
                    status="down",
                    error=f"HTTP {response.status_code}",
                )
    except Exception as e:
        logger.error(f"OpenRouter health check failed: {e}")
        dependencies["openrouter"] = DependencyStatus(
            status="down", error=str(e)
        )

    # Determine overall status
    statuses = [dep.status for dep in dependencies.values()]
    if all(s == "up" for s in statuses):
        overall_status = "healthy"
    elif any(s == "down" for s in statuses):
        overall_status = "unhealthy"
    else:
        overall_status = "degraded"

    response = HealthResponse(
        status=overall_status,
        timestamp=datetime.utcnow(),
        dependencies=dependencies,
    )

    # Return 503 if unhealthy
    if overall_status == "unhealthy":
        raise HTTPException(status_code=503, detail=response.dict())

    return response
