"""
Qdrant Cloud vector database client implementation.
Manages vector embeddings for semantic search.
"""

import logging
from typing import List, Optional, Dict, Any

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    SearchRequest,
    Filter,
    FieldCondition,
    MatchValue,
)

logger = logging.getLogger(__name__)


class QdrantVectorClient:
    """Qdrant Cloud client for vector operations."""

    COLLECTION_NAME = "book_chunks"
    VECTOR_SIZE = 768  # Qwen3 Embedding 8B dimension
    DISTANCE_METRIC = Distance.COSINE

    def __init__(self, qdrant_url: str, qdrant_api_key: str):
        """
        Initialize Qdrant client.

        Args:
            qdrant_url: Qdrant Cloud cluster URL
            qdrant_api_key: API key for authentication
        """
        self.qdrant_url = qdrant_url
        self.qdrant_api_key = qdrant_api_key
        self.client: Optional[QdrantClient] = None

    async def connect(self):
        """Initialize Qdrant client and create collection if needed."""
        try:
            self.client = QdrantClient(
                url=self.qdrant_url,
                api_key=self.qdrant_api_key,
                timeout=30,
            )

            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_exists = any(c.name == self.COLLECTION_NAME for c in collections)

            if not collection_exists:
                logger.info(f"Creating collection: {self.COLLECTION_NAME}")
                self.client.create_collection(
                    collection_name=self.COLLECTION_NAME,
                    vectors_config=VectorParams(
                        size=self.VECTOR_SIZE,
                        distance=self.DISTANCE_METRIC,
                    ),
                )
                logger.info(f"Collection {self.COLLECTION_NAME} created successfully")
            else:
                logger.info(f"Collection {self.COLLECTION_NAME} already exists")

        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            raise

    async def disconnect(self):
        """Close Qdrant client connection."""
        if self.client:
            self.client.close()
            logger.info("Qdrant client closed")

    async def upsert_vectors(
        self, chunk_ids: List[str], vectors: List[List[float]], payloads: List[Dict[str, Any]]
    ):
        """
        Insert or update vectors in Qdrant.

        Args:
            chunk_ids: List of chunk UUIDs
            vectors: List of embedding vectors (768 dimensions)
            payloads: List of metadata dictionaries
        """
        if not self.client:
            raise RuntimeError("Qdrant client not initialized")

        if len(chunk_ids) != len(vectors) != len(payloads):
            raise ValueError("chunk_ids, vectors, and payloads must have the same length")

        try:
            points = [
                PointStruct(id=chunk_id, vector=vector, payload=payload)
                for chunk_id, vector, payload in zip(chunk_ids, vectors, payloads)
            ]

            self.client.upsert(collection_name=self.COLLECTION_NAME, points=points)

            logger.info(f"Upserted {len(points)} vectors to Qdrant")
        except Exception as e:
            logger.error(f"Failed to upsert vectors: {e}")
            raise

    async def search_similar(
        self,
        query_vector: List[float],
        limit: int = 10,
        score_threshold: float = 0.0,
        chapter_filter: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors using cosine similarity.

        Args:
            query_vector: Query embedding (768 dimensions)
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score (0.0-1.0)
            chapter_filter: Optional chapter name filter

        Returns:
            List of dicts with chunk_id, score, and payload
        """
        if not self.client:
            raise RuntimeError("Qdrant client not initialized")

        try:
            # Build filter if chapter specified
            search_filter = None
            if chapter_filter:
                search_filter = Filter(
                    must=[FieldCondition(key="chapter_name", match=MatchValue(value=chapter_filter))]
                )

            # Perform search
            results = self.client.search(
                collection_name=self.COLLECTION_NAME,
                query_vector=query_vector,
                limit=limit,
                score_threshold=score_threshold,
                query_filter=search_filter,
            )

            # Format results
            formatted_results = [
                {
                    "chunk_id": result.id,
                    "relevance_score": result.score,
                    "chapter_name": result.payload.get("chapter_name"),
                    "section_name": result.payload.get("section_name"),
                    "text": result.payload.get("text"),
                    "document_path": result.payload.get("document_path"),
                }
                for result in results
            ]

            logger.info(f"Found {len(formatted_results)} similar chunks")
            return formatted_results

        except Exception as e:
            logger.error(f"Vector search failed: {e}")
            raise

    async def delete_collection(self):
        """Delete the entire collection (use with caution)."""
        if not self.client:
            raise RuntimeError("Qdrant client not initialized")

        try:
            self.client.delete_collection(collection_name=self.COLLECTION_NAME)
            logger.info(f"Collection {self.COLLECTION_NAME} deleted")
        except Exception as e:
            logger.error(f"Failed to delete collection: {e}")
            raise

    async def health_check(self) -> dict:
        """
        Check Qdrant health and return status.

        Returns:
            dict: Health status with latency and collection info
        """
        import time

        try:
            start_time = time.time()
            collections = self.client.get_collections()
            latency_ms = int((time.time() - start_time) * 1000)

            # Get collection info
            collection_exists = any(c.name == self.COLLECTION_NAME for c in collections.collections)
            vector_count = 0
            if collection_exists:
                collection_info = self.client.get_collection(collection_name=self.COLLECTION_NAME)
                vector_count = collection_info.vectors_count

            return {
                "status": "up",
                "latency_ms": latency_ms,
                "collection_exists": collection_exists,
                "vector_count": vector_count,
                "error": None,
            }
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return {"status": "down", "latency_ms": None, "error": str(e)}


# Global instance (initialized in main.py)
qdrant_client: Optional[QdrantVectorClient] = None


def get_qdrant_client() -> QdrantVectorClient:
    """Get the global Qdrant client instance."""
    if qdrant_client is None:
        raise RuntimeError("Qdrant client not initialized")
    return qdrant_client
