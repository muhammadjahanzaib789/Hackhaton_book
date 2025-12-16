"""
Chat logging service for managing sessions and storing query/response records.
Handles ChatSession creation, Query logging, Response logging, and SourceCitation tracking.
"""

import logging
from datetime import datetime, timedelta
from typing import List, Optional
from uuid import UUID, uuid4

from ..db.neon_client import get_neon_client
from ..models.session import SessionMode, SessionStatus
from ..models.query import Query
from ..models.response import Response
from ..models.citation import SourceCitation
from .retrieval import RetrievalResult

logger = logging.getLogger(__name__)


class ChatLoggerService:
    """Service for logging chat interactions and managing sessions."""

    async def create_session(
        self, mode: SessionMode, user_id: Optional[str] = None
    ) -> UUID:
        """
        Create a new chat session.

        Args:
            mode: Session mode (full_book or selected_text)
            user_id: Optional user identifier

        Returns:
            UUID of the created session

        Raises:
            Exception: If session creation fails
        """
        try:
            session_id = uuid4()
            neon_client = get_neon_client()

            async with neon_client.get_connection() as conn:
                await conn.execute(
                    """
                    INSERT INTO chat_sessions (session_id, mode, user_id, status, started_at)
                    VALUES ($1, $2, $3, $4, $5)
                    """,
                    session_id,
                    mode.value,
                    user_id,
                    SessionStatus.ACTIVE.value,
                    datetime.utcnow(),
                )

            logger.info(f"Created session {session_id} with mode {mode.value}")
            return session_id

        except Exception as e:
            logger.error(f"Failed to create session: {e}", exc_info=True)
            raise

    async def log_query(
        self,
        session_id: UUID,
        query_text: str,
        mode: SessionMode,
        selected_text: Optional[str] = None,
    ) -> UUID:
        """
        Log a user query.

        Args:
            session_id: Session UUID
            query_text: User's question
            mode: Session mode
            selected_text: Optional selected text (required for selected_text mode)

        Returns:
            UUID of the logged query

        Raises:
            Exception: If query logging fails
        """
        try:
            query_id = uuid4()
            neon_client = get_neon_client()

            async with neon_client.get_connection() as conn:
                await conn.execute(
                    """
                    INSERT INTO queries (query_id, session_id, query_text, mode, selected_text, created_at)
                    VALUES ($1, $2, $3, $4, $5, $6)
                    """,
                    query_id,
                    session_id,
                    query_text,
                    mode.value,
                    selected_text,
                    datetime.utcnow(),
                )

            logger.info(f"Logged query {query_id} for session {session_id}")
            return query_id

        except Exception as e:
            logger.error(f"Failed to log query: {e}", exc_info=True)
            raise

    async def log_response(
        self,
        query_id: UUID,
        answer_text: str,
        model_name: str,
        processing_time_ms: int,
    ) -> UUID:
        """
        Log an LLM-generated response with telemetry data.

        Args:
            query_id: Query UUID
            answer_text: Generated answer
            model_name: LLM model used
            processing_time_ms: Total processing time

        Returns:
            UUID of the logged response

        Raises:
            Exception: If response logging fails
        """
        try:
            response_id = uuid4()
            neon_client = get_neon_client()

            # Calculate token count for analytics
            answer_token_count = len(answer_text.split())  # Rough estimate

            async with neon_client.get_connection() as conn:
                await conn.execute(
                    """
                    INSERT INTO responses (response_id, query_id, answer_text, model_name, processing_time_ms, token_count, created_at)
                    VALUES ($1, $2, $3, $4, $5, $6, $7)
                    """,
                    response_id,
                    query_id,
                    answer_text,
                    model_name,
                    processing_time_ms,
                    answer_token_count,
                    datetime.utcnow(),
                )

            # Log telemetry for analytics
            logger.info(
                f"Response logged | response_id={response_id} | query_id={query_id} | "
                f"model={model_name} | processing_time_ms={processing_time_ms} | "
                f"answer_tokens={answer_token_count}"
            )
            return response_id

        except Exception as e:
            logger.error(f"Failed to log response: {e}", exc_info=True)
            raise

    async def log_source_citations(
        self, response_id: UUID, retrieved_chunks: List[RetrievalResult]
    ):
        """
        Log source citations for a response with navigation links and excerpts.

        Args:
            response_id: Response UUID
            retrieved_chunks: List of RetrievalResult objects (should be pre-sorted by relevance)

        Raises:
            Exception: If citation logging fails
        """
        try:
            if not retrieved_chunks:
                logger.warning(f"No citations to log for response {response_id}")
                return

            neon_client = get_neon_client()

            async with neon_client.get_connection() as conn:
                # Batch insert citations (already sorted by relevance in query endpoint)
                for idx, chunk in enumerate(retrieved_chunks):
                    citation_id = uuid4()

                    # Generate excerpt (first 500 chars)
                    excerpt = chunk.text[:500] if len(chunk.text) > 500 else chunk.text

                    # Generate navigation link to Docusaurus route
                    link = self._generate_docusaurus_link(
                        chunk.document_path, chunk.section_name
                    )

                    await conn.execute(
                        """
                        INSERT INTO source_citations
                        (citation_id, response_id, chunk_id, relevance_score,
                         chapter_name, section_name, page_number, document_path,
                         citation_order, excerpt, link)
                        VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11)
                        """,
                        citation_id,
                        response_id,
                        chunk.chunk_id,
                        chunk.relevance_score,
                        chunk.chapter_name,
                        chunk.section_name,
                        chunk.page_number,
                        chunk.document_path,
                        idx + 1,  # 1-indexed citation order
                        excerpt,
                        link,
                    )

            logger.info(
                f"Logged {len(retrieved_chunks)} source citations for response {response_id}"
            )

        except Exception as e:
            logger.error(f"Failed to log source citations: {e}", exc_info=True)
            raise

    def _generate_docusaurus_link(
        self, document_path: Optional[str], section_name: Optional[str]
    ) -> Optional[str]:
        """
        Generate a Docusaurus route from document_path and section_name.

        Args:
            document_path: File path like "/path/to/docs/chapter-01/lesson-01.md"
            section_name: Section name for anchor

        Returns:
            Docusaurus route like "/docs/chapter-01/lesson-01#section-name"
        """
        if not document_path:
            return None

        try:
            # Extract the relevant part of the path
            # Expected format: .../docs/chapter-XX/lesson-YY.md or similar
            import re
            from pathlib import Path

            path = Path(document_path)

            # Find "docs" directory and extract relative path
            parts = path.parts
            try:
                docs_index = parts.index("docs")
                # Get parts after "docs"
                relative_parts = parts[docs_index + 1 :]

                # Remove .md extension from last part
                if relative_parts:
                    last_part = relative_parts[-1].replace(".md", "")
                    route_parts = list(relative_parts[:-1]) + [last_part]

                    # Build Docusaurus route: /docs/chapter-XX/lesson-YY
                    route = "/docs/" + "/".join(route_parts)

                    # Add section anchor if available
                    if section_name:
                        # Convert section name to URL-safe anchor
                        anchor = re.sub(r"[^\w\s-]", "", section_name.lower())
                        anchor = re.sub(r"[\s_]+", "-", anchor)
                        route += f"#{anchor}"

                    return route

            except ValueError:
                # "docs" not in path parts
                logger.warning(f"Could not find 'docs' in path: {document_path}")
                return None

        except Exception as e:
            logger.error(f"Failed to generate Docusaurus link: {e}")
            return None

    async def update_session_status(
        self, session_id: UUID, status: SessionStatus
    ):
        """
        Update session status.

        Args:
            session_id: Session UUID
            status: New session status

        Raises:
            Exception: If status update fails
        """
        try:
            neon_client = get_neon_client()

            async with neon_client.get_connection() as conn:
                # Update status and ended_at if ending the session
                if status in [SessionStatus.ENDED, SessionStatus.ABANDONED]:
                    await conn.execute(
                        """
                        UPDATE chat_sessions
                        SET status = $1, ended_at = $2
                        WHERE session_id = $3
                        """,
                        status.value,
                        datetime.utcnow(),
                        session_id,
                    )
                else:
                    await conn.execute(
                        """
                        UPDATE chat_sessions
                        SET status = $1
                        WHERE session_id = $3
                        """,
                        status.value,
                        session_id,
                    )

            logger.info(f"Updated session {session_id} status to {status.value}")

        except Exception as e:
            logger.error(f"Failed to update session status: {e}", exc_info=True)
            raise

    async def get_session_history(
        self, session_id: UUID, limit: int = 10
    ) -> List[dict]:
        """
        Retrieve query/response history for a session.

        Args:
            session_id: Session UUID
            limit: Maximum number of Q&A pairs to retrieve

        Returns:
            List of dicts with query_text, answer_text, created_at

        Raises:
            Exception: If retrieval fails
        """
        try:
            neon_client = get_neon_client()

            async with neon_client.get_connection() as conn:
                rows = await conn.fetch(
                    """
                    SELECT q.query_text, r.answer_text, q.created_at
                    FROM queries q
                    LEFT JOIN responses r ON q.query_id = r.query_id
                    WHERE q.session_id = $1
                    ORDER BY q.created_at DESC
                    LIMIT $2
                    """,
                    session_id,
                    limit,
                )

                history = []
                for row in rows:
                    history.append(
                        {
                            "query_text": row["query_text"],
                            "answer_text": row["answer_text"],
                            "created_at": row["created_at"],
                        }
                    )

                logger.info(
                    f"Retrieved {len(history)} history items for session {session_id}"
                )
                return history

        except Exception as e:
            logger.error(f"Failed to retrieve session history: {e}", exc_info=True)
            raise

    async def get_analytics_summary(
        self, days: int = 7
    ) -> dict:
        """
        Get analytics summary for the specified time period.

        Args:
            days: Number of days to include in summary (default 7)

        Returns:
            Dict with analytics data:
            - total_queries: Total number of queries
            - total_sessions: Total number of sessions
            - avg_processing_time_ms: Average processing time
            - mode_distribution: Queries by mode
            - top_chapters: Most queried chapters

        Raises:
            Exception: If analytics retrieval fails
        """
        try:
            neon_client = get_neon_client()
            cutoff_date = datetime.utcnow() - timedelta(days=days)

            async with neon_client.get_connection() as conn:
                # Total queries
                total_queries = await conn.fetchval(
                    """
                    SELECT COUNT(*) FROM queries
                    WHERE created_at >= $1
                    """,
                    cutoff_date,
                )

                # Total sessions
                total_sessions = await conn.fetchval(
                    """
                    SELECT COUNT(DISTINCT session_id) FROM queries
                    WHERE created_at >= $1
                    """,
                    cutoff_date,
                )

                # Average processing time
                avg_processing_time = await conn.fetchval(
                    """
                    SELECT AVG(r.processing_time_ms)
                    FROM responses r
                    JOIN queries q ON r.query_id = q.query_id
                    WHERE q.created_at >= $1
                    """,
                    cutoff_date,
                )

                # Mode distribution
                mode_rows = await conn.fetch(
                    """
                    SELECT mode, COUNT(*) as count
                    FROM queries
                    WHERE created_at >= $1
                    GROUP BY mode
                    """,
                    cutoff_date,
                )
                mode_distribution = {row["mode"]: row["count"] for row in mode_rows}

                # Top chapters (from citations)
                chapter_rows = await conn.fetch(
                    """
                    SELECT sc.chapter_name, COUNT(*) as count
                    FROM source_citations sc
                    JOIN responses r ON sc.response_id = r.response_id
                    JOIN queries q ON r.query_id = q.query_id
                    WHERE q.created_at >= $1
                    GROUP BY sc.chapter_name
                    ORDER BY count DESC
                    LIMIT 10
                    """,
                    cutoff_date,
                )
                top_chapters = [
                    {"chapter": row["chapter_name"], "citations": row["count"]}
                    for row in chapter_rows
                ]

                analytics = {
                    "period_days": days,
                    "total_queries": total_queries,
                    "total_sessions": total_sessions,
                    "avg_processing_time_ms": int(avg_processing_time) if avg_processing_time else 0,
                    "mode_distribution": mode_distribution,
                    "top_chapters": top_chapters,
                }

                logger.info(f"Retrieved analytics summary: {analytics}")
                return analytics

        except Exception as e:
            logger.error(f"Failed to retrieve analytics: {e}", exc_info=True)
            raise


# Global instance
chat_logger_service = ChatLoggerService()


def get_chat_logger_service() -> ChatLoggerService:
    """Get the global chat logger service instance."""
    return chat_logger_service
