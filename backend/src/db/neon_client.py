"""
Neon Postgres connection pool implementation.
Uses psycopg3 with connection pooling for efficient database access.
"""

import logging
from contextlib import asynccontextmanager
from typing import Optional

import psycopg
from psycopg_pool import ConnectionPool

logger = logging.getLogger(__name__)


class NeonClient:
    """Neon Serverless Postgres client with connection pooling."""

    def __init__(self, database_url: str, min_size: int = 2, max_size: int = 10):
        """
        Initialize Neon Postgres connection pool.

        Args:
            database_url: PostgreSQL connection string
            min_size: Minimum number of connections in pool
            max_size: Maximum number of connections in pool
        """
        self.database_url = database_url
        self.pool: Optional[ConnectionPool] = None
        self.min_size = min_size
        self.max_size = max_size

    async def connect(self):
        """Initialize the connection pool."""
        try:
            # Create connection pool with async connections
            self.pool = ConnectionPool(
                conninfo=self.database_url,
                min_size=self.min_size,
                max_size=self.max_size,
                timeout=30,  # Connection timeout in seconds
            )

            # Test connection
            async with self.pool.connection() as conn:
                await conn.execute("SELECT 1")

            logger.info(
                f"Neon Postgres pool initialized: {self.min_size}-{self.max_size} connections"
            )
        except Exception as e:
            logger.error(f"Failed to connect to Neon Postgres: {e}")
            raise

    async def disconnect(self):
        """Close the connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("Neon Postgres pool closed")

    @asynccontextmanager
    async def get_connection(self):
        """
        Context manager for getting a database connection from the pool.

        Usage:
            async with neon_client.get_connection() as conn:
                result = await conn.execute("SELECT * FROM table")
        """
        if not self.pool:
            raise RuntimeError("Connection pool not initialized. Call connect() first.")

        async with self.pool.connection() as conn:
            try:
                yield conn
            except Exception as e:
                logger.error(f"Database operation failed: {e}")
                raise

    async def health_check(self) -> dict:
        """
        Check database health and return status.

        Returns:
            dict: Health status with latency and connection info
        """
        import time

        try:
            start_time = time.time()
            async with self.get_connection() as conn:
                await conn.execute("SELECT 1")
            latency_ms = int((time.time() - start_time) * 1000)

            return {
                "status": "up",
                "latency_ms": latency_ms,
                "pool_size": self.pool.get_stats()["pool_size"] if self.pool else 0,
                "error": None,
            }
        except Exception as e:
            logger.error(f"Neon health check failed: {e}")
            return {"status": "down", "latency_ms": None, "error": str(e)}

    async def run_migrations(self, migrations_dir: str = "src/db/migrations"):
        """
        Run database migrations from SQL files.

        Args:
            migrations_dir: Directory containing SQL migration files
        """
        import os
        from pathlib import Path

        migrations_path = Path(migrations_dir)
        if not migrations_path.exists():
            logger.warning(f"Migrations directory not found: {migrations_dir}")
            return

        # Get all .sql files sorted by name
        migration_files = sorted(migrations_path.glob("*.sql"))

        if not migration_files:
            logger.info("No migration files found")
            return

        async with self.get_connection() as conn:
            for migration_file in migration_files:
                logger.info(f"Running migration: {migration_file.name}")
                try:
                    sql_content = migration_file.read_text()
                    await conn.execute(sql_content)
                    logger.info(f"Migration {migration_file.name} completed successfully")
                except Exception as e:
                    logger.error(f"Migration {migration_file.name} failed: {e}")
                    raise


# Global instance (initialized in main.py)
neon_client: Optional[NeonClient] = None


def get_neon_client() -> NeonClient:
    """Get the global Neon client instance."""
    if neon_client is None:
        raise RuntimeError("Neon client not initialized")
    return neon_client
