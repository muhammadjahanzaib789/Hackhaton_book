"""
Unit tests for the retrieval service in the RAG Retrieval Validation feature.
"""

import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from typing import List

from src.services.retrieval_service import RetrievalService
from src.models.retrieval import Query, RetrievalResult


class TestRetrievalService:
    """Unit tests for the RetrievalService class."""
    
    @pytest.fixture
    def mock_qdrant_service(self):
        """Mock Qdrant service for testing."""
        mock = AsyncMock()
        mock.search = AsyncMock()
        return mock
    
    @pytest.fixture
    def mock_embedding_service(self):
        """Mock embedding service for testing."""
        mock = AsyncMock()
        mock.generate_embedding = AsyncMock()
        return mock
    
    @pytest.fixture
    def retrieval_service(self, mock_qdrant_service, mock_embedding_service):
        """Create a retrieval service instance with mocked dependencies."""
        service = RetrievalService()
        service.qdrant_service = mock_qdrant_service
        service.embedding_service = mock_embedding_service
        return service
    
    @pytest.mark.asyncio
    async def test_retrieve_success(self, retrieval_service, mock_qdrant_service, mock_embedding_service):
        """Test successful retrieval with valid query and results."""
        # Setup mock responses
        mock_embedding_service.generate_embedding.return_value = MagicMock(
            vector=[0.1, 0.2, 0.3] * 341 + [0.4]  # 1024-dimensional vector
        )
        
        mock_qdrant_service.search.return_value = [
            MagicMock(
                chunk_id="chunk-1",
                text="Test document text",
                url="http://example.com",
                section="Introduction",
                chunk_index=0,
                similarity_score=0.8,
                timestamp=None
            )
        ]
        
        # Create test query
        query = Query(
            query_text="What is AI?",
            top_k=5,
            similarity_threshold=0.5
        )
        
        # Execute retrieval
        result = await retrieval_service.retrieve(query)
        
        # Validate results
        assert result is not None
        assert result.status == "success"
        assert len(result.results) == 1
        assert result.results[0].chunk_id == "chunk-1"
        assert result.results[0].text == "Test document text"
        assert result.results[0].url == "http://example.com"
        assert result.results[0].similarity_score == 0.8
        
        # Verify that the services were called with correct parameters
        mock_embedding_service.generate_embedding.assert_called_once_with("What is AI?")
        mock_qdrant_service.search.assert_called_once_with(
            query_embedding=[0.1, 0.2, 0.3] * 341 + [0.4],
            top_k=5,
            similarity_threshold=0.5,
            filters=None
        )
    
    @pytest.mark.asyncio
    async def test_retrieve_with_filters(self, retrieval_service, mock_qdrant_service, mock_embedding_service):
        """Test retrieval with metadata filters."""
        # Setup mock responses
        mock_embedding_service.generate_embedding.return_value = MagicMock(
            vector=[0.1, 0.2, 0.3] * 341 + [0.4]  # 1024-dimensional vector
        )
        
        mock_qdrant_service.search.return_value = [
            MagicMock(
                chunk_id="chunk-2",
                text="Filtered document text",
                url="http://example.com/doc2",
                section="Advanced",
                chunk_index=1,
                similarity_score=0.9,
                timestamp=None
            )
        ]
        
        # Create test query with filters
        query = Query(
            query_text="Advanced concepts",
            top_k=3,
            similarity_threshold=0.6,
            filters={"section": "Advanced", "url": "http://example.com/doc2"}
        )
        
        # Execute retrieval
        result = await retrieval_service.retrieve(query)
        
        # Validate results
        assert result is not None
        assert result.status == "success"
        assert len(result.results) == 1
        assert result.results[0].chunk_id == "chunk-2"
        assert result.results[0].section == "Advanced"
        assert result.results[0].similarity_score == 0.9
        
        # Verify that the services were called with correct parameters including filters
        mock_embedding_service.generate_embedding.assert_called_once_with("Advanced concepts")
        mock_qdrant_service.search.assert_called_once_with(
            query_embedding=[0.1, 0.2, 0.3] * 341 + [0.4],
            top_k=3,
            similarity_threshold=0.6,
            filters={"section": "Advanced", "url": "http://example.com/doc2"}
        )
    
    @pytest.mark.asyncio
    async def test_retrieve_no_results(self, retrieval_service, mock_qdrant_service, mock_embedding_service):
        """Test retrieval when no results are returned from Qdrant."""
        # Setup mock responses
        mock_embedding_service.generate_embedding.return_value = MagicMock(
            vector=[0.1, 0.2, 0.3] * 341 + [0.4]  # 1024-dimensional vector
        )
        
        mock_qdrant_service.search.return_value = []  # No results found
        
        # Create test query
        query = Query(
            query_text="Very specific query with no matches",
            top_k=5,
            similarity_threshold=0.5
        )
        
        # Execute retrieval
        result = await retrieval_service.retrieve(query)
        
        # Validate results
        assert result is not None
        assert result.status == "partial"
        assert len(result.results) == 0
        
        # Verify that the services were called
        mock_embedding_service.generate_embedding.assert_called_once()
        mock_qdrant_service.search.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_retrieve_embedding_error(self, retrieval_service, mock_qdrant_service, mock_embedding_service):
        """Test retrieval when embedding generation fails."""
        # Setup mock to raise an exception
        mock_embedding_service.generate_embedding.side_effect = Exception("Embedding API Error")
        
        # Create test query
        query = Query(
            query_text="Query that causes embedding error",
            top_k=5,
            similarity_threshold=0.5
        )
        
        # Execute retrieval and expect an exception
        with pytest.raises(Exception) as exc_info:
            await retrieval_service.retrieve(query)
        
        # Verify the exception message
        assert "Embedding API Error" in str(exc_info.value)
        
        # Verify that embedding service was called but Qdrant search was not
        mock_embedding_service.generate_embedding.assert_called_once_with("Query that causes embedding error")
        mock_qdrant_service.search.assert_not_called()
    
    @pytest.mark.asyncio
    async def test_retrieve_qdrant_error(self, retrieval_service, mock_qdrant_service, mock_embedding_service):
        """Test retrieval when Qdrant search fails."""
        # Setup mock responses
        mock_embedding_service.generate_embedding.return_value = MagicMock(
            vector=[0.1, 0.2, 0.3] * 341 + [0.4]  # 1024-dimensional vector
        )
        
        # Setup mock to raise an exception during search
        mock_qdrant_service.search.side_effect = Exception("Qdrant Error")
        
        # Create test query
        query = Query(
            query_text="Query that causes Qdrant error",
            top_k=5,
            similarity_threshold=0.5
        )
        
        # Execute retrieval and expect an exception
        with pytest.raises(Exception) as exc_info:
            await retrieval_service.retrieve(query)
        
        # Verify the exception message
        assert "Qdrant Error" in str(exc_info.value)
        
        # Verify that both services were called
        mock_embedding_service.generate_embedding.assert_called_once()
        mock_qdrant_service.search.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_batch_retrieve(self, retrieval_service, mock_qdrant_service, mock_embedding_service):
        """Test batch retrieval functionality."""
        # Setup mock responses
        mock_embedding_service.generate_embedding.return_value = MagicMock(
            vector=[0.1, 0.2, 0.3] * 341 + [0.4]  # 1024-dimensional vector
        )
        
        mock_qdrant_service.search.return_value = [
            MagicMock(
                chunk_id="chunk-1",
                text="Test document text",
                url="http://example.com",
                section="Introduction",
                chunk_index=0,
                similarity_score=0.8,
                timestamp=None
            )
        ]
        
        # Create test queries
        queries = [
            Query(query_text="First query", top_k=5, similarity_threshold=0.5),
            Query(query_text="Second query", top_k=5, similarity_threshold=0.5)
        ]
        
        # Execute batch retrieval
        results = await retrieval_service.batch_retrieve(queries)
        
        # Validate results
        assert len(results) == 2
        for result in results:
            assert result.status == "success"
            assert len(result.results) == 1
            assert result.results[0].chunk_id == "chunk-1"
    
    @pytest.mark.asyncio
    async def test_batch_retrieve_with_error(self, retrieval_service, mock_qdrant_service, mock_embedding_service):
        """Test batch retrieval when one query fails."""
        # Setup mock responses - first call succeeds, second fails
        embedding_calls = 0
        def side_effect(text):
            nonlocal embedding_calls
            embedding_calls += 1
            if embedding_calls == 2:  # Second call fails
                raise Exception("Embedding Error")
            return MagicMock(vector=[0.1, 0.2, 0.3] * 341 + [0.4])
        
        mock_embedding_service.generate_embedding.side_effect = side_effect
        mock_qdrant_service.search.return_value = [
            MagicMock(
                chunk_id="chunk-1",
                text="Test document text",
                url="http://example.com",
                section="Introduction",
                chunk_index=0,
                similarity_score=0.8,
                timestamp=None
            )
        ]
        
        # Create test queries
        queries = [
            Query(query_text="Good query", top_k=5, similarity_threshold=0.5),
            Query(query_text="Bad query", top_k=5, similarity_threshold=0.5)
        ]
        
        # Execute batch retrieval
        results = await retrieval_service.batch_retrieve(queries)
        
        # Validate results - first should succeed, second should fail
        assert len(results) == 2
        
        # First query should succeed
        assert results[0].status == "success"
        assert len(results[0].results) == 1
        
        # Second query should fail
        assert results[1].status == "error"
        assert results[1].error_message is not None
        assert "Embedding Error" in results[1].error_message