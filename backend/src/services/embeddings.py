"""
Embedding generation service using Qwen3 Embedding 8B via OpenRouter.
Uses OpenAI-compatible API with custom base_url.
"""

import logging
from typing import List
from openai import OpenAI

from ..config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class EmbeddingService:
    """Service for generating text embeddings using Qwen3 via OpenRouter."""

    def __init__(self):
        # Initialize OpenAI client with OpenRouter base URL
        # This pattern matches the user's provided code snippet
        self.client = OpenAI(
            base_url=settings.openrouter_base_url,
            api_key=settings.openrouter_api_key,
        )
        self.model = settings.embedding_model  # qwen/qwen3-embedding-8b

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text string.

        Args:
            text: Text to embed

        Returns:
            768-dimensional embedding vector

        Raises:
            Exception: If embedding generation fails
        """
        try:
            # Generate embedding using OpenRouter
            response = self.client.embeddings.create(
                model=self.model,
                input=text,
            )

            # Extract embedding vector
            embedding = response.data[0].embedding

            # Validate dimensions (should be 768 for Qwen3 Embedding 8B)
            if len(embedding) != 768:
                logger.warning(
                    f"Expected 768 dimensions, got {len(embedding)} for model {self.model}"
                )

            return embedding

        except Exception as e:
            logger.error(f"Embedding generation failed: {e}")
            raise

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batch.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors

        Note: OpenRouter may have rate limits, handle accordingly
        """
        try:
            # Generate embeddings in batch
            response = self.client.embeddings.create(
                model=self.model,
                input=texts,
            )

            # Extract all embeddings
            embeddings = [item.embedding for item in response.data]

            logger.info(f"Generated {len(embeddings)} embeddings in batch")
            return embeddings

        except Exception as e:
            logger.error(f"Batch embedding generation failed: {e}")
            # Fallback: generate one by one
            logger.info("Falling back to individual embedding generation")
            embeddings = []
            for text in texts:
                try:
                    embedding = await self.generate_embedding(text)
                    embeddings.append(embedding)
                except Exception as embed_error:
                    logger.error(f"Failed to embed text: {embed_error}")
                    # Use zero vector as fallback
                    embeddings.append([0.0] * 768)

            return embeddings


# Global instance
embedding_service = EmbeddingService()


def get_embedding_service() -> EmbeddingService:
    """Get the global embedding service instance."""
    return embedding_service
