"""
Configuration management for RAG Chatbot backend.
Loads settings from environment variables.
"""

import os
from typing import List
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

    # OpenRouter API Configuration
    openrouter_api_key: str
    openrouter_base_url: str = "https://openrouter.ai/api/v1"

    # Qdrant Cloud Configuration
    qdrant_url: str
    qdrant_api_key: str

    # Neon Serverless Postgres Configuration
    database_url: str

    # Application Settings
    environment: str = "development"
    log_level: str = "INFO"

    # CORS Configuration
    cors_origins: str = "http://localhost:3000,http://localhost:3001"

    # Chunking Parameters
    chunk_size: int = 400
    chunk_overlap: int = 80
    min_chunk_size: int = 100
    max_chunks_retrieved: int = 10

    # LLM Configuration
    llm_model: str = "openai/gpt-4-turbo-preview"
    embedding_model: str = "qwen/qwen3-embedding-8b"
    llm_temperature: float = 0.7
    llm_max_tokens: int = 2000
    llm_timeout: int = 30  # seconds

    # Database Connection Pool
    db_pool_min_size: int = 2
    db_pool_max_size: int = 10

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",") if origin.strip()]

    @property
    def is_production(self) -> bool:
        """Check if running in production environment."""
        return self.environment.lower() == "production"

    @property
    def is_development(self) -> bool:
        """Check if running in development environment."""
        return self.environment.lower() == "development"


# Global settings instance
_settings: Settings | None = None


def get_settings() -> Settings:
    """
    Get application settings singleton.

    Returns:
        Settings: Application configuration
    """
    global _settings
    if _settings is None:
        _settings = Settings()
    return _settings


# Convenience function for direct import
settings = get_settings()
