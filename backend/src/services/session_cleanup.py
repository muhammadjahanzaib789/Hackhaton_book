"""
Session cleanup service to mark abandoned sessions.
Runs as a background task to update session status based on inactivity.
"""

import logging
from datetime import datetime, timedelta
from typing import List
from uuid import UUID

from ..db.neon_client import get_neon_client
from ..models.session import SessionStatus

logger = logging.getLogger(__name__)

# Session timeout: 30 minutes of inactivity
SESSION_TIMEOUT_MINUTES = 30


class SessionCleanupService:
    """Service for cleaning up abandoned chat sessions."""

    async def cleanup_abandoned_sessions(self) -> int:
        """
        Mark sessions as abandoned if inactive for > 30 minutes.

        A session is considered abandoned if:
        - Status is 'active'
        - No queries in the last 30 minutes
        - Not already marked as 'ended' or 'abandoned'

        Returns:
            Number of sessions marked as abandoned
        """
        try:
            neon_client = get_neon_client()
            timeout_threshold = datetime.utcnow() - timedelta(
                minutes=SESSION_TIMEOUT_MINUTES
            )

            async with neon_client.get_connection() as conn:
                # Find active sessions with no recent queries
                rows = await conn.fetch(
                    """
                    SELECT DISTINCT s.session_id
                    FROM chat_sessions s
                    LEFT JOIN queries q ON s.session_id = q.session_id
                    WHERE s.status = $1
                    AND (
                        q.created_at IS NULL
                        OR q.created_at < $2
                    )
                    GROUP BY s.session_id, s.started_at
                    HAVING MAX(COALESCE(q.created_at, s.started_at)) < $2
                    """,
                    SessionStatus.ACTIVE.value,
                    timeout_threshold,
                )

                if not rows:
                    logger.info("No abandoned sessions found")
                    return 0

                session_ids = [row["session_id"] for row in rows]

                # Mark sessions as abandoned
                result = await conn.execute(
                    """
                    UPDATE chat_sessions
                    SET status = $1, ended_at = $2
                    WHERE session_id = ANY($3)
                    """,
                    SessionStatus.ABANDONED.value,
                    datetime.utcnow(),
                    session_ids,
                )

                count = len(session_ids)
                logger.info(f"Marked {count} sessions as abandoned")
                return count

        except Exception as e:
            logger.error(f"Session cleanup failed: {e}", exc_info=True)
            return 0

    async def get_session_stats(self) -> dict:
        """
        Get statistics about session status distribution.

        Returns:
            Dict with counts per status
        """
        try:
            neon_client = get_neon_client()

            async with neon_client.get_connection() as conn:
                rows = await conn.fetch(
                    """
                    SELECT status, COUNT(*) as count
                    FROM chat_sessions
                    GROUP BY status
                    """
                )

                stats = {row["status"]: row["count"] for row in rows}
                logger.info(f"Session stats: {stats}")
                return stats

        except Exception as e:
            logger.error(f"Failed to get session stats: {e}", exc_info=True)
            return {}


# Global instance
session_cleanup_service = SessionCleanupService()


def get_session_cleanup_service() -> SessionCleanupService:
    """Get the global session cleanup service instance."""
    return session_cleanup_service
