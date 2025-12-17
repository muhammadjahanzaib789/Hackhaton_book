"""
Retrieval service for semantic search over book content.
Performs vector similarity search in Qdrant and enriches with metadata from Neon Postgres.
"""

import logging
import hashlib
from typing import List, Dict, Any, Optional
from uuid import UUID, uuid4
from functools import lru_cache
from datetime import datetime, timedelta

from ..db.qdrant_client import get_qdrant_client
from ..db.neon_client import get_neon_client
from ..config import get_settings
from .embeddings import EmbeddingService

logger = logging.getLogger(__name__)
settings = get_settings()


# Simple in-memory cache for query embeddings
class QueryCache:
    """LRU cache for query embeddings with TTL."""

    def __init__(self, max_size: int = 100, ttl_seconds: int = 3600):
        self.cache: Dict[str, tuple[List[float], datetime]] = {}
        self.max_size = max_size
        self.ttl = timedelta(seconds=ttl_seconds)

    def _make_key(self, query: str, selected_text: Optional[str] = None) -> str:
        """Generate cache key from query and selected_text."""
        text = query + (selected_text or "")
        return hashlib.md5(text.encode()).hexdigest()

    def get(self, query: str, selected_text: Optional[str] = None) -> Optional[List[float]]:
        """Get cached embedding if exists and not expired."""
        key = self._make_key(query, selected_text)
        if key in self.cache:
            embedding, timestamp = self.cache[key]
            if datetime.now() - timestamp < self.ttl:
                logger.debug(f"Cache hit for query: {query[:50]}...")
                return embedding
            else:
                # Expired, remove from cache
                del self.cache[key]
        return None

    def set(self, query: str, embedding: List[float], selected_text: Optional[str] = None):
        """Cache embedding with current timestamp."""
        key = self._make_key(query, selected_text)

        # Evict oldest entry if cache is full
        if len(self.cache) >= self.max_size:
            oldest_key = min(self.cache.keys(), key=lambda k: self.cache[k][1])
            del self.cache[oldest_key]
            logger.debug(f"Evicted oldest cache entry (cache size: {self.max_size})")

        self.cache[key] = (embedding, datetime.now())
        logger.debug(f"Cached embedding for query: {query[:50]}...")


class RetrievalResult:
    """Single retrieval result with chunk content and metadata."""

    def __init__(
        self,
        chunk_id: UUID,
        text: str,
        relevance_score: float,
        chapter_name: str,
        section_name: Optional[str] = None,
        page_number: Optional[int] = None,
        document_path: Optional[str] = None,
    ):
        self.chunk_id = chunk_id
        self.text = text
        self.relevance_score = relevance_score
        self.chapter_name = chapter_name
        self.section_name = section_name
        self.page_number = page_number
        self.document_path = document_path

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for response serialization."""
        return {
            "chunk_id": str(self.chunk_id),
            "text": self.text,
            "relevance_score": self.relevance_score,
            "chapter_name": self.chapter_name,
            "section_name": self.section_name,
            "page_number": self.page_number,
            "document_path": self.document_path,
        }


class RetrievalService:
    """Service for retrieving relevant book content using semantic search."""

    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.relevance_threshold = 0.3  # Minimum cosine similarity score
        self.query_cache = QueryCache(max_size=100, ttl_seconds=3600)  # 1-hour TTL

    async def retrieve(
        self,
        query: str,
        top_k: int = 5,
        score_threshold: Optional[float] = None,
        selected_text: Optional[str] = None,
        mode: Optional[str] = None,
    ) -> List[RetrievalResult]:
        """
        Retrieve relevant chunks for a query using semantic search.

        For selected_text mode, creates an in-memory chunk from the selected text
        instead of searching Qdrant, ensuring LLM only uses the selected passage.

        Args:
            query: User's question
            top_k: Number of top results to return (default 5)
            score_threshold: Minimum relevance score (default 0.3)
            selected_text: Optional selected text for selected_text mode
            mode: Query mode (full_book or selected_text)

        Returns:
            List of RetrievalResult objects sorted by relevance

        Raises:
            Exception: If retrieval fails
        """
        score_threshold = score_threshold or self.relevance_threshold

        try:
            # Special handling for selected_text mode
            if mode == "selected_text" and selected_text:
                logger.info("Using selected_text mode: creating in-memory chunk")
                return await self._create_selected_text_chunk(selected_text)

            # Full-book mode: semantic search in Qdrant
            # Generate query embedding (with caching for performance)
            query_text = query
            if selected_text:
                # Combine selected text with query for better context
                query_text = f"Context: {selected_text}\n\nQuestion: {query}"

            # Try cache first
            query_embedding = self.query_cache.get(query_text, selected_text)

            if query_embedding is None:
                # Cache miss: generate new embedding
                logger.info(f"Generating embedding for query: {query[:100]}...")
                query_embedding = await self.embedding_service.generate_embedding(
                    query_text
                )
                # Cache the result
                self.query_cache.set(query_text, query_embedding, selected_text)
            else:
                logger.info(f"Using cached embedding for query: {query[:100]}...")

            # Search Qdrant for similar vectors
            qdrant_client = get_qdrant_client()
            search_results = await qdrant_client.search_similar(
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=score_threshold,
            )

            logger.info(
                f"Qdrant returned {len(search_results)} results above threshold {score_threshold}"
            )

            if not search_results:
                logger.warning("No relevant chunks found above score threshold")
                return []

            # Extract chunk IDs for metadata lookup
            chunk_ids = [result["id"] for result in search_results]

            # Fetch metadata from Neon Postgres
            neon_client = get_neon_client()
            metadata_map = await self._fetch_chunk_metadata(neon_client, chunk_ids)

            # Combine Qdrant results with Neon metadata
            retrieval_results = []
            for result in search_results:
                chunk_id = result["id"]
                score = result["score"]
                payload = result["payload"]

                # Get metadata from Neon (or fallback to Qdrant payload)
                metadata = metadata_map.get(chunk_id, {})

                retrieval_result = RetrievalResult(
                    chunk_id=UUID(chunk_id),
                    text=payload.get("text") or metadata.get("text", ""),
                    relevance_score=score,
                    chapter_name=payload.get("chapter_name")
                    or metadata.get("chapter_name", "Unknown"),
                    section_name=payload.get("section_name")
                    or metadata.get("section_name"),
                    page_number=metadata.get("page_number"),
                    document_path=payload.get("document_path")
                    or metadata.get("document_path"),
                )
                retrieval_results.append(retrieval_result)

            logger.info(
                f"Retrieved {len(retrieval_results)} relevant chunks for query"
            )
            return retrieval_results

        except Exception as e:
            logger.error(f"Retrieval failed: {e}", exc_info=True)
            raise

    async def _create_selected_text_chunk(
        self, selected_text: str
    ) -> List[RetrievalResult]:
        """
        Create an in-memory chunk from selected text for selected_text mode.

        This ensures the LLM only uses the selected passage and doesn't search
        the entire book, enforcing strict constraint per User Story 2.

        Args:
            selected_text: The text selected by the user

        Returns:
            List with single RetrievalResult containing the selected text
        """
        try:
            # Create a single in-memory chunk from selected text
            chunk_id = uuid4()  # Temporary ID for in-memory chunk

            # Set high relevance score since this is the exact selected text
            relevance_score = 1.0

            # Create retrieval result
            result = RetrievalResult(
                chunk_id=chunk_id,
                text=selected_text,
                relevance_score=relevance_score,
                chapter_name="Selected Text",
                section_name="User Selection",
                page_number=None,
                document_path=None,
            )

            logger.info(
                f"Created in-memory chunk from selected text ({len(selected_text)} chars)"
            )
            return [result]

        except Exception as e:
            logger.error(f"Failed to create selected text chunk: {e}", exc_info=True)
            raise

    async def _fetch_chunk_metadata(
        self, neon_client, chunk_ids: List[str]
    ) -> Dict[str, Dict[str, Any]]:
        """
        Fetch chunk metadata from Neon Postgres.

        Args:
            neon_client: NeonClient instance
            chunk_ids: List of chunk UUIDs

        Returns:
            Dict mapping chunk_id to metadata dict
        """
        if not chunk_ids:
            return {}

        try:
            async with neon_client.get_connection() as conn:
                # Build parameterized query
                placeholders = ", ".join(
                    f"${i+1}" for i in range(len(chunk_ids))
                )
                query = f"""
                    SELECT chunk_id, text, chapter_name, section_name,
                           page_number, document_path, token_count
                    FROM book_content_chunks
                    WHERE chunk_id IN ({placeholders})
                """

                rows = await conn.fetch(query, *chunk_ids)

                # Build metadata map
                metadata_map = {}
                for row in rows:
                    metadata_map[str(row["chunk_id"])] = {
                        "text": row["text"],
                        "chapter_name": row["chapter_name"],
                        "section_name": row["section_name"],
                        "page_number": row["page_number"],
                        "document_path": row["document_path"],
                        "token_count": row["token_count"],
                    }

                logger.info(f"Fetched metadata for {len(metadata_map)} chunks")
                return metadata_map

        except Exception as e:
            logger.error(f"Failed to fetch chunk metadata: {e}")
            # Return empty map to allow fallback to Qdrant payload
            return {}


# Global instance
retrieval_service = RetrievalService()


def get_retrieval_service() -> RetrievalService:
    """Get the global retrieval service instance."""
    return retrieval_service
