"""
Admin endpoint for indexing book content.
Triggers the content indexing pipeline: parse markdown → chunk → embed → store.
"""

import logging
from typing import Optional

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field

from ...services.content_indexer import ContentIndexer

logger = logging.getLogger(__name__)
router = APIRouter()


class IndexRequest(BaseModel):
    """Request schema for indexing book content."""

    content_path: str = Field(
        ...,
        description="Path to the book content directory containing Markdown files",
        min_length=1,
    )
    force_reindex: bool = Field(
        default=False,
        description="If true, delete existing chunks before indexing",
    )

    class Config:
        json_schema_extra = {
            "example": {
                "content_path": "/path/to/book/content",
                "force_reindex": False,
            }
        }


class IndexResponse(BaseModel):
    """Response schema for indexing operation."""

    status: str = Field(
        ...,
        description="Status of indexing operation: success, partial_success, or failed",
    )
    chunks_indexed: int = Field(
        ..., description="Number of chunks successfully indexed"
    )
    failed_files: list[str] = Field(
        default_factory=list, description="List of files that failed to index"
    )
    processing_time_ms: int = Field(
        ..., description="Total processing time in milliseconds"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "status": "success",
                "chunks_indexed": 245,
                "failed_files": [],
                "processing_time_ms": 12450,
            }
        }


@router.post("/index", response_model=IndexResponse, status_code=status.HTTP_200_OK)
async def index_content(request: IndexRequest):
    """
    Index book content from a directory.

    Scans the specified directory for Markdown files, parses them,
    chunks the content, generates embeddings, and stores chunks in
    Qdrant (vectors) and Neon Postgres (metadata).

    Args:
        request: IndexRequest with content_path and force_reindex flag

    Returns:
        IndexResponse: Indexing statistics including chunks_indexed,
                       failed_files, and processing_time_ms

    Raises:
        400: Invalid content_path or directory not found
        500: Internal server error during indexing
    """
    logger.info(
        f"Index request received: path={request.content_path}, force_reindex={request.force_reindex}"
    )

    try:
        # Create indexer and process content
        indexer = ContentIndexer()
        result = await indexer.index_content(
            content_path=request.content_path,
            force_reindex=request.force_reindex,
        )

        # Validate result
        if result["status"] == "failed" and result["chunks_indexed"] == 0:
            logger.error(
                f"Indexing failed completely: {result.get('failed_files', [])}"
            )
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail={
                    "error": "INDEXING_FAILED",
                    "message": "Failed to index any content",
                    "failed_files": result.get("failed_files", []),
                },
            )

        # Log success or partial success
        if result["status"] == "partial_success":
            logger.warning(
                f"Partial success: {result['chunks_indexed']} chunks indexed, "
                f"{len(result.get('failed_files', []))} files failed"
            )
        else:
            logger.info(
                f"Indexing successful: {result['chunks_indexed']} chunks indexed "
                f"in {result['processing_time_ms']}ms"
            )

        # Return response
        return IndexResponse(**result)

    except FileNotFoundError as e:
        logger.error(f"Content directory not found: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": "DIRECTORY_NOT_FOUND",
                "message": str(e),
            },
        )

    except ValueError as e:
        logger.error(f"Invalid request: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": "INVALID_REQUEST",
                "message": str(e),
            },
        )

    except Exception as e:
        logger.error(f"Indexing error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "INDEXING_ERROR",
                "message": "An unexpected error occurred during indexing",
            },
        )
