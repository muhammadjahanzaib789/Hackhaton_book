"""
Core retrieval service for RAG Retrieval Validation.

Handles the main retrieval logic including query embedding,
Qdrant search, result ranking, and response formatting.
"""

import asyncio
import time
import logging
from typing import List, Optional, Dict, Any
from pydantic import BaseModel

from .qdrant_service import get_qdrant_service, QdrantSearchResult
from .embedding_service import get_cohere_embedding_service, EmbeddingResponse
from ..models.retrieval import Query, RetrievalResult, RetrievalResponse
from ..utils.logging import log_retrieval_request, log_retrieval_response
from ..utils.errors import RetrievalError, handle_retrieval_error

logger = logging.getLogger(__name__)


class RetrievalService:
    """Main service for executing retrieval operations."""
    
    def __init__(self):
        self.qdrant_service = get_qdrant_service()
        self.embedding_service = get_cohere_embedding_service()
    
    async def retrieve(self, query: Query) -> RetrievalResponse:
        """
        Execute a retrieval operation for the given query.
        
        Args:
            query: Query object containing search parameters
            
        Returns:
            RetrievalResponse with results and timing information
        """
        start_time = time.time()
        query_latency_breakdown = {
            'embedding_generation_time': 0,
            'qdrant_search_time': 0,
            'post_processing_time': 0
        }
        
        # Log the retrieval request
        log_retrieval_request(
            query_text=query.query_text,
            top_k=query.top_k,
            filters=query.filters
        )
        
        # Generate embedding for the query
        embedding_start = time.time()
        try:
            embedding_result: EmbeddingResponse = await self.embedding_service.generate_embedding(query.query_text)
            query_latency_breakdown['embedding_generation_time'] = time.time() - embedding_start
        except Exception as e:
            raise RetrievalError(
                message=f"Failed to generate embedding for query: {str(e)}",
                error_code="EMBEDDING_GENERATION_ERROR",
                details={"query_text": query.query_text}
            )
        
        # Perform search in Qdrant
        search_start = time.time()
        try:
            search_results: List[QdrantSearchResult] = await self.qdrant_service.search(
                query_embedding=embedding_result.vector,
                top_k=query.top_k,
                similarity_threshold=query.similarity_threshold,
                filters=query.filters
            )
            query_latency_breakdown['qdrant_search_time'] = time.time() - search_start
        except Exception as e:
            raise RetrievalError(
                message=f"Failed to search in Qdrant: {str(e)}",
                error_code="QDRANT_SEARCH_ERROR",
                details={"query_text": query.query_text}
            )
        
        # Process and format results
        post_processing_start = time.time()
        try:
            # Convert Qdrant results to our RetrievalResult format
            formatted_results = []
            for qdrant_result in search_results:
                formatted_results.append(RetrievalResult(
                    chunk_id=qdrant_result.chunk_id,
                    text=qdrant_result.text,
                    url=qdrant_result.url,
                    section=qdrant_result.section,
                    chunk_index=qdrant_result.chunk_index,
                    similarity_score=qdrant_result.similarity_score,
                    timestamp=qdrant_result.timestamp
                ))
            
            # Ensure results are sorted by similarity_score in descending order
            formatted_results.sort(key=lambda x: x.similarity_score, reverse=True)
            
            # Apply similarity threshold filtering after processing
            filtered_results = [
                result for result in formatted_results
                if result.similarity_score >= query.similarity_threshold
            ]
            
            query_latency_breakdown['post_processing_time'] = time.time() - post_processing_start
            
            # Calculate total latency
            total_latency = time.time() - start_time
            
            # Prepare the response
            response = RetrievalResponse(
                query_id=query.query_id or f"retrieval-{int(time.time())}",
                results=filtered_results,
                query_latency=total_latency,
                embedding_generation_time=query_latency_breakdown['embedding_generation_time'],
                qdrant_search_time=query_latency_breakdown['qdrant_search_time'],
                post_processing_time=query_latency_breakdown['post_processing_time'],
                status="success" if filtered_results else "partial",
                error_message=None
            )
            
            # Log the retrieval response
            log_retrieval_response(
                query_id=response.query_id,
                result_count=len(filtered_results),
                query_latency=total_latency
            )
            
            return response
            
        except Exception as e:
            raise RetrievalError(
                message=f"Failed to process retrieval results: {str(e)}",
                error_code="RESULT_PROCESSING_ERROR",
                details={
                    "query_text": query.query_text,
                    "num_search_results": len(search_results)
                }
            )
    
    async def retrieve_with_timing(self, query: Query) -> RetrievalResponse:
        """
        Execute retrieval with detailed timing breakdown.
        
        Args:
            query: Query object containing search parameters
            
        Returns:
            RetrievalResponse with detailed timing information
        """
        start_time = time.time()
        
        # Perform the actual retrieval
        response = await self.retrieve(query)
        
        # Add total query latency to the response
        response.query_latency = time.time() - start_time
        
        return response
    
    async def batch_retrieve(self, queries: List[Query]) -> List[RetrievalResponse]:
        """
        Execute multiple retrieval operations in batch.
        
        Args:
            queries: List of Query objects to process
            
        Returns:
            List of RetrievalResponse objects
        """
        responses = []
        for query in queries:
            try:
                response = await self.retrieve(query)
                responses.append(response)
            except Exception as e:
                # Create an error response for failed queries
                error_response = RetrievalResponse(
                    query_id=query.query_id or f"error-{int(time.time())}",
                    results=[],
                    query_latency=0,
                    embedding_generation_time=0,
                    qdrant_search_time=0,
                    post_processing_time=0,
                    status="error",
                    error_message=str(e)
                )
                responses.append(error_response)
        
        return responses


# Global instance
retrieval_service: Optional[RetrievalService] = None


def get_retrieval_service() -> RetrievalService:
    """Get the global retrieval service instance."""
    global retrieval_service
    if retrieval_service is None:
        retrieval_service = RetrievalService()
    return retrieval_service