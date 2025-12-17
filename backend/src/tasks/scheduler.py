"""
Background task scheduler for periodic maintenance tasks.
Uses asyncio to run tasks at specified intervals.
"""

import asyncio
import logging
from typing import Optional

from ..services.session_cleanup import get_session_cleanup_service

logger = logging.getLogger(__name__)


class TaskScheduler:
    """Scheduler for background maintenance tasks."""

    def __init__(self):
        self.running = False
        self.task: Optional[asyncio.Task] = None

    async def start(self):
        """Start the background task scheduler."""
        if self.running:
            logger.warning("Task scheduler already running")
            return

        self.running = True
        self.task = asyncio.create_task(self._run_scheduler())
        logger.info("Background task scheduler started")

    async def stop(self):
        """Stop the background task scheduler."""
        if not self.running:
            return

        self.running = False
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
        logger.info("Background task scheduler stopped")

    async def _run_scheduler(self):
        """Main scheduler loop."""
        cleanup_service = get_session_cleanup_service()

        while self.running:
            try:
                # Run session cleanup every 10 minutes
                logger.info("Running scheduled session cleanup")
                abandoned_count = await cleanup_service.cleanup_abandoned_sessions()
                logger.info(f"Session cleanup completed: {abandoned_count} sessions marked as abandoned")

                # Get session stats for monitoring
                stats = await cleanup_service.get_session_stats()
                logger.info(f"Current session stats: {stats}")

                # Wait 10 minutes before next run
                await asyncio.sleep(600)  # 10 minutes

            except asyncio.CancelledError:
                logger.info("Scheduler task cancelled")
                break
            except Exception as e:
                logger.error(f"Error in scheduler: {e}", exc_info=True)
                # Wait 1 minute before retrying on error
                await asyncio.sleep(60)


# Global scheduler instance
task_scheduler = TaskScheduler()


def get_task_scheduler() -> TaskScheduler:
    """Get the global task scheduler instance."""
    return task_scheduler
