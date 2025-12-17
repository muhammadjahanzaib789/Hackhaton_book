"""
Cohere Embedding Service for RAG Retrieval Validation

This service handles generating embeddings using the Cohere API,
specifically for query embedding generation that matches the 
embedding space used during ingestion.
"""

import asyncio
import logging
import time
from typing import List, Optional
from contextlib import contextmanager
import cohere
from pydantic import BaseModel

from ..config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class EmbeddingResponse(BaseModel):
    """Response model for embedding generation."""
    vector: List[float]
    model: str
    generation_timestamp: float


class CohereEmbeddingService:
    """Service for generating text embeddings using Cohere API."""
    
    def __init__(self):
        """Initialize the Cohere client with API key from settings."""
        api_key = settings.cohere_api_key if hasattr(settings, 'cohere_api_key') else None
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        
        self.client = cohere.Client(api_key)
        self.model = getattr(settings, 'cohere_model', 'embed-english-v3.0')
        
        # Constants from the spec
        self.batch_size = getattr(settings, 'cohere_batch_size', 96)
        self.dimension = 1024  # Cohere embed-english-v3.0 produces 1024-dim vectors

    async def generate_embedding(self, text: str) -> EmbeddingResponse:
        """
        Generate embedding for a single text string.
        
        Args:
            text: Text to embed (non-empty)
            
        Returns:
            EmbeddingResponse containing the 1024-dimensional vector
            
        Raises:
            ValueError: If text is empty
            Exception: If embedding generation fails
        """
        if not text or not text.strip():
            raise ValueError("Text must not be empty")
        
        try:
            # Call Cohere API to generate embedding
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type="search_query",  # Using search_query for queries
                embedding_types=["float"]
            )
            
            # Extract the embedding vector
            embedding_vector = response.embeddings.float_[0]
            
            # Validate dimensions
            if len(embedding_vector) != self.dimension:
                raise ValueError(f"Expected {self.dimension} dimensions, got {len(embedding_vector)}")
            
            return EmbeddingResponse(
                vector=embedding_vector,
                model=self.model,
                generation_timestamp=time.time()
            )
        except Exception as e:
            logger.error(f"Failed to generate embedding for text: {str(e)}")
            raise

    async def generate_embeddings_batch(self, texts: List[str]) -> List[EmbeddingResponse]:
        """
        Generate embeddings for multiple texts in batch.
        
        Args:
            texts: List of texts to embed (non-empty)
            
        Returns:
            List of EmbeddingResponse objects
        """
        if not texts:
            return []
        
        # Validate all texts are non-empty
        for i, text in enumerate(texts):
            if not text or not text.strip():
                raise ValueError(f"Text at index {i} must not be empty")
        
        # Process in batches to respect API limits
        all_embeddings = []
        for i in range(0, len(texts), self.batch_size):
            batch = texts[i:i + self.batch_size]
            
            try:
                # Generate embeddings for this batch
                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type="search_query",  # Using search_query for queries
                    embedding_types=["float"]
                )
                
                # Process each embedding result in the batch
                for j, embedding_vector in enumerate(response.embeddings.float_):
                    if len(embedding_vector) != self.dimension:
                        raise ValueError(f"Expected {self.dimension} dimensions, got {len(embedding_vector)}")
                    
                    all_embeddings.append(EmbeddingResponse(
                        vector=embedding_vector,
                        model=self.model,
                        generation_timestamp=time.time()
                    ))
            except Exception as e:
                logger.error(f"Failed to generate embeddings for batch starting at index {i}: {str(e)}")
                # On failure, try individual embeddings as fallback
                for text in batch:
                    try:
                        single_embedding = await self.generate_embedding(text)
                        all_embeddings.append(single_embedding)
                    except Exception as individual_error:
                        logger.error(f"Failed to generate embedding for single text: {str(individual_error)}")
                        # Add zero vector as fallback
                        all_embeddings.append(EmbeddingResponse(
                            vector=[0.0] * self.dimension,
                            model=self.model,
                            generation_timestamp=time.time()
                        ))
        
        return all_embeddings


# Global instance
cohere_embedding_service: Optional[CohereEmbeddingService] = None


def get_cohere_embedding_service() -> CohereEmbeddingService:
    """Get the global Cohere embedding service instance."""
    global cohere_embedding_service
    if cohere_embedding_service is None:
        cohere_embedding_service = CohereEmbeddingService()
    return cohere_embedding_service