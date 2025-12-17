"""
API endpoints for the RAG Retrieval Validation feature.

This module provides endpoints for semantic search and retrieval operations.
"""

import asyncio
import time
import logging
from typing import Optional
from fastapi import APIRouter, HTTPException, status, Depends
from fastapi.responses import JSONResponse

from ..models.retrieval import Query, RetrievalResponse, BatchQueryRequest, BatchQueryResponse
from ..services.retrieval_service import get_retrieval_service
from ..utils.errors import handle_retrieval_error, ErrorResponse
from ..utils.logging import log_execution_time

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post("/retrieval/query", response_model=RetrievalResponse, status_code=status.HTTP_200_OK)
@log_execution_time
async def query_retrieval(query_request: Query):
    """
    Execute semantic search query against vector database.
    
    Accepts a natural language query and returns semantically similar document chunks.
    
    Args:
        query_request: Query object with search parameters
        
    Returns:
        RetrievalResponse with ranked results
    """
    try:
        logger.info(f"Received retrieval query: {query_request.query_text[:100]}...")
        
        # Get retrieval service instance
        retrieval_svc = get_retrieval_service()
        
        # Execute the retrieval
        start_time = time.time()
        response = await retrieval_svc.retrieve(query_request)
        total_time = time.time() - start_time
        
        logger.info(f"Retrieval completed in {total_time:.4f}s with {len(response.results)} results")
        return response
        
    except Exception as e:
        logger.error(f"Error in query_retrieval: {str(e)}")
        return handle_retrieval_error(e, query_request.query_text)


@router.post("/retrieval/batch", response_model=BatchQueryResponse, status_code=status.HTTP_200_OK)
@log_execution_time
async def batch_query_retrieval(batch_request: BatchQueryRequest):
    """
    Execute batch semantic search queries.
    
    Processes multiple queries in a single request.
    
    Args:
        batch_request: BatchQueryRequest with multiple queries
        
    Returns:
        BatchQueryResponse with individual responses for each query
    """
    try:
        logger.info(f"Received batch query with {len(batch_request.queries)} requests")
        
        # Get retrieval service instance
        retrieval_svc = get_retrieval_service()
        
        # Execute batch retrieval
        start_time = time.time()
        individual_responses = await retrieval_svc.batch_retrieve(batch_request.queries)
        total_time = time.time() - start_time
        
        # Create batch response
        batch_response = BatchQueryResponse(
            batch_id=batch_request.batch_id or f"batch-{int(time.time())}",
            responses=individual_responses,
            batch_latency=total_time
        )
        
        logger.info(f"Batch retrieval completed in {total_time:.4f}s")
        return batch_response
        
    except Exception as e:
        logger.error(f"Error in batch_query_retrieval: {str(e)}")
        # Handle error appropriately
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error during batch retrieval",
                "error_code": "INTERNAL_ERROR"
            }
        )


# Validation endpoint will be added in a separate file