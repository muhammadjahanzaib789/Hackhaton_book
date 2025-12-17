"""
Qdrant Service for RAG Retrieval Validation

This service handles interactions with the Qdrant vector database,
specifically for semantic search and metadata filtering operations.
"""

import asyncio
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct, VectorParams, Distance
from pydantic import BaseModel

from ..config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class QdrantSearchResult(BaseModel):
    """Model for Qdrant search results."""
    chunk_id: str
    text: str
    url: str
    section: Optional[str]
    chunk_index: int
    similarity_score: float
    timestamp: Optional[str]
    content_hash: Optional[str]


class QdrantService:
    """Service for interacting with Qdrant vector database."""
    
    def __init__(self):
        """Initialize the Qdrant client with connection parameters."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = getattr(settings, 'qdrant_collection_name', 'rag_embedding')
        
    async def search(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        similarity_threshold: float = 0.5,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[QdrantSearchResult]:
        """
        Perform semantic search in Qdrant with optional metadata filtering.
        
        Args:
            query_embedding: 1024-dimensional embedding vector for the query
            top_k: Number of top results to return
            similarity_threshold: Minimum similarity score for results
            filters: Optional metadata filters (e.g., {'url': '...', 'section': '...'})
            
        Returns:
            List of QdrantSearchResult objects sorted by similarity score
        """
        try:
            # Prepare filters for Qdrant
            qdrant_filters = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    if isinstance(value, list):
                        # Handle array of possible values with 'should' condition
                        should_conditions = []
                        for val in value:
                            should_conditions.append(
                                models.FieldCondition(
                                    key=key,
                                    match=models.MatchValue(value=val)
                                )
                            )
                        conditions.append(models.Should(should_conditions=should_conditions))
                    else:
                        # Handle single value with 'must' condition
                        conditions.append(
                            models.FieldCondition(
                                key=key,
                                match=models.MatchValue(value=value)
                            )
                        )
                
                if conditions:
                    qdrant_filters = models.Filter(must=conditions)
            
            # Perform the search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=qdrant_filters,
                limit=top_k,
                score_threshold=similarity_threshold,
                with_payload=True,
                with_vectors=False
            )
            
            # Convert Qdrant results to our model format
            results = []
            for hit in search_results:
                payload = hit.payload
                results.append(QdrantSearchResult(
                    chunk_id=str(hit.id),
                    text=payload.get('text', ''),
                    url=payload.get('url', ''),
                    section=payload.get('section', None),
                    chunk_index=payload.get('chunk_index', 0),
                    similarity_score=hit.score,
                    timestamp=payload.get('indexed_at', None),
                    content_hash=payload.get('content_hash', None)
                ))
            
            logger.info(f"Qdrant search returned {len(results)} results")
            return results
            
        except Exception as e:
            logger.error(f"Qdrant search failed: {str(e)}")
            raise
    
    async def validate_connection(self) -> bool:
        """
        Validate connection to Qdrant and check if collection exists.
        
        Returns:
            True if connection is valid and collection exists, False otherwise
        """
        try:
            # Get list of collections to check if our collection exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]
            
            if self.collection_name not in collection_names:
                logger.error(f"Collection '{self.collection_name}' does not exist in Qdrant")
                return False
            
            logger.info(f"Successfully connected to Qdrant collection '{self.collection_name}'")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {str(e)}")
            return False
    
    async def count_points(self) -> int:
        """
        Get the total number of points in the collection.
        
        Returns:
            Total number of points in the collection
        """
        try:
            count = self.client.count(
                collection_name=self.collection_name
            )
            return count.count
        except Exception as e:
            logger.error(f"Failed to count points in Qdrant: {str(e)}")
            raise


# Global instance
qdrant_service: Optional[QdrantService] = None


def get_qdrant_service() -> QdrantService:
    """Get the global Qdrant service instance."""
    global qdrant_service
    if qdrant_service is None:
        qdrant_service = QdrantService()
    return qdrant_service