"""
Agent service module for the OpenAI RAG Agent feature.

This module contains the core agent orchestration service that handles
query processing, response generation, and integration with the retrieval pipeline.
"""

import asyncio
import logging
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
import openai
from openai import OpenAI
from openai import APIError, APIConnectionError, RateLimitError, AuthenticationError, NotFoundError, ConflictError, UnprocessableEntityError
from pydantic import BaseModel

from ..models.agent import (
    AgentRequest, AgentResponse, SourceCitation,
    RetrievalToolRequest, RetrievalToolResponse, RetrievedChunk, ErrorResponse
)
from ..config import get_settings
from ..services.retrieval_service import RetrievalService
from ..utils.agent_logging import get_agent_logger, log_agent_query_start, log_agent_query_end, log_agent_retrieval, log_agent_error, log_agent_validation
from ..utils.agent_errors import AgentError, AgentRetrievalError, AgentValidationError, AgentErrorCode, handle_agent_error


class AgentService:
    """
    Core service class for the RAG Agent system.

    This service orchestrates the interaction between the OpenAI Agent,
    the retrieval pipeline, and response generation to provide grounded
    answers based on the knowledge base.
    """

    def __init__(self):
        """Initialize the AgentService with required dependencies."""
        self.logger = get_agent_logger(__name__)
        self.settings = get_settings()
        self.retrieval_service = RetrievalService()

        # Initialize OpenAI client
        self.openai_client = OpenAI(api_key=self.settings.openai_api_key)

        # Agent configuration
        self.agent_id = self.settings.openai_agent_id
        self.model = self.settings.openai_model or "gpt-4-turbo-preview"

        # Initialize metrics tracking
        self.metrics = {
            "total_queries": 0,
            "successful_queries": 0,
            "failed_queries": 0,
            "total_retrieval_time": 0.0,
            "total_agent_time": 0.0,
            "total_queries_time": 0.0,
            "total_processing_time": 0.0,  # Total time including all steps
            "query_count_by_hour": {},
            "avg_response_time": 0.0,
            "avg_tokens_per_response": 0,
            "avg_retrieved_chunks": 0,
            "avg_grounding_score": 0.0,
            "avg_retrieval_time": 0.0,
            "avg_agent_processing_time": 0.0,
            "avg_citation_relevance_score": 0.0,
            "queries_by_model": {},
            "queries_by_temperature": {},
            "queries_by_max_tokens": {},
            "error_types": {},
            "success_rate_by_hour": {},
            "p95_response_time": 0.0,
            "p99_response_time": 0.0
        }

        self.logger.info("AgentService initialized successfully")

    def _update_metrics(self, query_time: float, tokens_used: Optional[int] = None,
                       retrieved_chunks: Optional[int] = None, grounding_score: Optional[float] = None,
                       retrieval_time: Optional[float] = None, agent_processing_time: Optional[float] = None,
                       avg_citation_relevance: Optional[float] = None, agent_request: Optional[AgentRequest] = None) -> None:
        """
        Update performance metrics based on query results.

        Args:
            query_time: Time taken for the complete query in seconds
            tokens_used: Number of tokens in the response
            retrieved_chunks: Number of chunks retrieved
            grounding_score: Grounding score of the response
            retrieval_time: Time taken for retrieval in seconds
            agent_processing_time: Time taken for agent processing in seconds
            avg_citation_relevance: Average relevance score of citations
            agent_request: The original agent request for parameter tracking
        """
        try:
            import time
            from collections import defaultdict

            # Increment counters
            self.metrics["total_queries"] += 1
            self.metrics["total_queries_time"] += query_time
            self.metrics["total_processing_time"] += query_time

            # Update hourly query count
            current_hour = int(time.time() // 3600)
            self.metrics["query_count_by_hour"][current_hour] = self.metrics["query_count_by_hour"].get(current_hour, 0) + 1

            # Track model usage
            if agent_request and agent_request.temperature is not None:
                temp_key = str(agent_request.temperature)
                self.metrics["queries_by_temperature"][temp_key] = self.metrics["queries_by_temperature"].get(temp_key, 0) + 1

            if agent_request and agent_request.max_tokens is not None:
                max_tokens_key = str(agent_request.max_tokens)
                self.metrics["queries_by_max_tokens"][max_tokens_key] = self.metrics["queries_by_max_tokens"].get(max_tokens_key, 0) + 1

            # Update tokens metric if provided
            if tokens_used is not None:
                total_tokens = self.metrics.get("total_tokens", 0) + tokens_used
                self.metrics["total_tokens"] = total_tokens
                count = self.metrics["successful_queries"] + 1
                self.metrics["avg_tokens_per_response"] = total_tokens / count

            # Update retrieved chunks metric if provided
            if retrieved_chunks is not None:
                total_chunks = self.metrics.get("total_retrieved_chunks", 0) + retrieved_chunks
                self.metrics["total_retrieved_chunks"] = total_chunks
                count = self.metrics["successful_queries"] + 1
                self.metrics["avg_retrieved_chunks"] = total_chunks / count

            # Update grounding score metric if provided
            if grounding_score is not None:
                total_grounding_score = self.metrics.get("total_grounding_score", 0.0) + grounding_score
                self.metrics["total_grounding_score"] = total_grounding_score
                count = self.metrics["successful_queries"] + 1
                self.metrics["avg_grounding_score"] = total_grounding_score / count

            # Update retrieval time metric if provided
            if retrieval_time is not None:
                self.metrics["total_retrieval_time"] += retrieval_time
                self.metrics["avg_retrieval_time"] = self.metrics["total_retrieval_time"] / self.metrics["total_queries"]

            # Update agent processing time metric if provided
            if agent_processing_time is not None:
                self.metrics["total_agent_time"] += agent_processing_time
                self.metrics["avg_agent_processing_time"] = self.metrics["total_agent_time"] / self.metrics["total_queries"]

            # Update citation relevance metric if provided
            if avg_citation_relevance is not None:
                total_citation_relevance = self.metrics.get("total_citation_relevance", 0.0) + avg_citation_relevance
                self.metrics["total_citation_relevance"] = total_citation_relevance
                count = self.metrics["successful_queries"] + 1
                self.metrics["avg_citation_relevance_score"] = total_citation_relevance / count

            # Update average response time
            self.metrics["avg_response_time"] = self.metrics["total_queries_time"] / self.metrics["total_queries"]

            self.logger.debug("Metrics updated", extra={
                "total_queries": self.metrics["total_queries"],
                "avg_response_time": self.metrics["avg_response_time"],
                "avg_retrieval_time": self.metrics.get("avg_retrieval_time", 0),
                "avg_agent_processing_time": self.metrics.get("avg_agent_processing_time", 0)
            })
        except Exception as e:
            self.logger.error(f"Error updating metrics: {str(e)}", exc_info=True)

    def _increment_failure_count(self) -> None:
        """Increment the failure counter."""
        self.metrics["failed_queries"] += 1

    def _increment_success_count(self) -> None:
        """Increment the success counter."""
        self.metrics["successful_queries"] += 1

    async def get_performance_metrics(self) -> Dict[str, Any]:
        """
        Get current performance metrics for the agent service.

        Returns:
            Dictionary containing performance metrics
        """
        try:
            # Calculate additional derived metrics
            total_queries = self.metrics["total_queries"]
            successful_queries = self.metrics["successful_queries"]
            failed_queries = self.metrics["failed_queries"]

            success_rate = (successful_queries / total_queries * 100) if total_queries > 0 else 0
            failure_rate = (failed_queries / total_queries * 100) if total_queries > 0 else 0

            # Clean up old hourly data (keep only last 24 hours)
            import time
            current_hour = int(time.time() // 3600)
            cutoff_hour = current_hour - 24
            self.metrics["query_count_by_hour"] = {
                hour: count for hour, count in self.metrics["query_count_by_hour"].items()
                if hour > cutoff_hour
            }

            # Calculate queries per hour
            recent_queries = sum(self.metrics["query_count_by_hour"].values())
            queries_per_hour = recent_queries / min(24, len(self.metrics["query_count_by_hour"])) if self.metrics["query_count_by_hour"] else 0

            metrics_report = {
                "summary": {
                    "total_queries": total_queries,
                    "successful_queries": successful_queries,
                    "failed_queries": failed_queries,
                    "success_rate": round(success_rate, 2),
                    "failure_rate": round(failure_rate, 2),
                    "queries_per_hour": round(queries_per_hour, 2)
                },
                "timing": {
                    "avg_response_time": round(self.metrics["avg_response_time"], 3),
                    "avg_retrieval_time": round(self.metrics.get("avg_retrieval_time", 0), 3),
                    "avg_agent_processing_time": round(self.metrics.get("avg_agent_processing_time", 0), 3),
                    "total_queries_time": round(self.metrics["total_queries_time"], 3),
                    "total_processing_time": round(self.metrics["total_processing_time"], 3)
                },
                "content_metrics": {
                    "avg_tokens_per_response": self.metrics["avg_tokens_per_response"],
                    "avg_retrieved_chunks": self.metrics["avg_retrieved_chunks"],
                    "avg_grounding_score": round(self.metrics["avg_grounding_score"], 3) if self.metrics["avg_grounding_score"] else 0.0,
                    "avg_citation_relevance_score": round(self.metrics["avg_citation_relevance_score"], 3) if self.metrics.get("avg_citation_relevance_score") else 0.0
                },
                "parameter_usage": {
                    "queries_by_temperature": self.metrics["queries_by_temperature"],
                    "queries_by_max_tokens": self.metrics["queries_by_max_tokens"]
                },
                "error_metrics": {
                    "error_types": self.metrics.get("error_types", {})
                },
                "timestamp": datetime.now().isoformat()
            }

            self.logger.info("Performance metrics retrieved", extra={"metrics": metrics_report["summary"]})
            return metrics_report

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"operation": "get_performance_metrics"}
            )
            log_agent_error(self.logger, error, "get_performance_metrics")
            raise error

    async def process_query(self, agent_request: AgentRequest) -> AgentResponse:
        """
        Process an agent query and return a grounded response.

        This method orchestrates the entire query processing pipeline:
        1. Creates a thread for the query
        2. Runs the agent with the retrieval tool
        3. Processes the response and extracts citations
        4. Returns a properly formatted AgentResponse

        Args:
            agent_request: The query and parameters from the user

        Returns:
            AgentResponse: The agent's response with citations and metadata
        """
        import uuid
        query_id = str(uuid.uuid4())
        start_time = datetime.now()

        # Additional validation beyond Pydantic model validation
        if not agent_request.query.strip():
            raise AgentValidationError("Query cannot be empty or whitespace only")

        if agent_request.max_tokens is not None and agent_request.max_tokens <= 0:
            raise AgentValidationError("max_tokens must be a positive integer")

        if agent_request.temperature is not None and (agent_request.temperature < 0.0 or agent_request.temperature > 2.0):
            raise AgentValidationError("temperature must be between 0.0 and 2.0")

        log_agent_query_start(
            self.logger,
            agent_request.query,
            query_id
        )

        try:
            # Create a new thread for this query with error handling
            try:
                thread = self.openai_client.beta.threads.create()
                thread_id = thread.id
            except RateLimitError as e:
                self.logger.warning(f"OpenAI API rate limit exceeded when creating thread: {str(e)}", extra={
                    "query_id": query_id
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RATE_LIMIT_ERROR,
                    message="Rate limit exceeded. Please try again later.",
                    details={"original_error": str(e)}
                )
            except AuthenticationError as e:
                self.logger.error(f"OpenAI API authentication error when creating thread: {str(e)}", extra={
                    "query_id": query_id
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_AUTHENTICATION_ERROR,
                    message="Authentication failed. Please check your API key.",
                    details={"original_error": str(e)}
                )
            except APIConnectionError as e:
                self.logger.error(f"OpenAI API connection error when creating thread: {str(e)}", extra={
                    "query_id": query_id
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_CONNECTION_ERROR,
                    message="Connection error occurred. Please try again.",
                    details={"original_error": str(e)}
                )
            except APIError as e:
                self.logger.error(f"OpenAI API error when creating thread: {str(e)}", extra={
                    "query_id": query_id,
                    "error_type": type(e).__name__
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RUNTIME_ERROR,
                    message=f"Error creating thread: {str(e)}",
                    details={"original_error": str(e)}
                )

            # Add the user's message to the thread
            try:
                self.openai_client.beta.threads.messages.create(
                    thread_id=thread_id,
                    role="user",
                    content=agent_request.query
                )
            except RateLimitError as e:
                self.logger.warning(f"OpenAI API rate limit exceeded when adding message to thread: {str(e)}", extra={
                    "thread_id": thread_id,
                    "query_id": query_id
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RATE_LIMIT_ERROR,
                    message="Rate limit exceeded. Please try again later.",
                    details={"original_error": str(e)}
                )
            except APIError as e:
                self.logger.error(f"OpenAI API error when adding message to thread: {str(e)}", extra={
                    "thread_id": thread_id,
                    "query_id": query_id,
                    "error_type": type(e).__name__
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RUNTIME_ERROR,
                    message=f"Error adding message to thread: {str(e)}",
                    details={"original_error": str(e)}
                )

            # Run the agent with the retrieval tool
            try:
                run = self.openai_client.beta.threads.runs.create(
                    thread_id=thread_id,
                    assistant_id=self.agent_id,
                    # Include the retrieval tool if available
                    tools=[{
                        "type": "function",
                        "function": {
                            "name": "retrieve_knowledge_base",
                            "description": "Retrieve relevant information from the knowledge base",
                            "parameters": {
                                "type": "object",
                                "properties": {
                                    "query": {"type": "string", "description": "The query to search for in the knowledge base"}
                                },
                                "required": ["query"]
                            }
                        }
                    }] if self.agent_id else []  # Only add tools if agent ID is configured
                )
            except RateLimitError as e:
                self.logger.warning(f"OpenAI API rate limit exceeded when starting run: {str(e)}", extra={
                    "thread_id": thread_id,
                    "query_id": query_id
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RATE_LIMIT_ERROR,
                    message="Rate limit exceeded. Please try again later.",
                    details={"original_error": str(e)}
                )
            except APIError as e:
                self.logger.error(f"OpenAI API error when starting run: {str(e)}", extra={
                    "thread_id": thread_id,
                    "query_id": query_id,
                    "error_type": type(e).__name__
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RUNTIME_ERROR,
                    message=f"Error starting agent run: {str(e)}",
                    details={"original_error": str(e)}
                )

            # Wait for the run to complete
            run = self._wait_for_run_completion(thread_id, run.id)

            # Get the messages from the thread
            messages = self.openai_client.beta.threads.messages.list(
                thread_id=thread_id,
                order="asc"
            )

            # Extract the agent's response
            agent_response_content = ""
            for message in messages.data:
                if message.role == "assistant":
                    for content_block in message.content:
                        if content_block.type == "text":
                            agent_response_content += content_block.text.value

            # Retrieve relevant chunks from the knowledge base with timing
            import time
            retrieval_start_time = time.time()
            retrieved_chunks = await self._retrieve_chunks_for_query(agent_request.query)
            retrieval_time = time.time() - retrieval_start_time

            # Generate citations from retrieved chunks
            citations = self._generate_citations_from_chunks(retrieved_chunks)

            # Adjust citation scores based on additional relevance calculations
            citations = self._adjust_citation_scores(citations, agent_request.query, agent_response_content)

            # Validate the grounding of the response
            grounding_validation = await self._validate_response_grounding(
                agent_request.query,
                agent_response_content,
                retrieved_chunks
            )

            # Calculate response time
            response_time = (datetime.now() - start_time).total_seconds()

            # Format the response with citations for proper integration
            # This enhances the agent's response with properly formatted citations
            formatted_answer = self._format_citations_for_response(citations, agent_response_content)

            # Create the final response with grounding confidence
            response = AgentResponse(
                answer=formatted_answer,
                citations=citations,
                query=agent_request.query,
                tokens_used=len(agent_response_content.split()),  # Approximate token count
                confidence=grounding_validation.get('grounding_score', 0.5)  # Use grounding score as confidence
            )

            # Calculate average citation relevance
            avg_citation_relevance = 0.0
            if citations:
                avg_citation_relevance = sum(citation.relevance_score for citation in citations) / len(citations)

            # Update metrics for successful query with detailed timing
            self._increment_success_count()
            self._update_metrics(
                query_time=response_time,
                tokens_used=len(agent_response_content.split()),
                retrieved_chunks=len(retrieved_chunks),
                grounding_score=grounding_validation.get('grounding_score', 0.5),
                retrieval_time=retrieval_time,
                avg_citation_relevance=avg_citation_relevance,
                agent_request=agent_request
            )

            log_agent_query_end(
                self.logger,
                query_id,
                len(agent_response_content),
                response_time,
                tokens_used=len(agent_response_content.split()),
                retrieved_chunks=len(retrieved_chunks),
                thread_id=thread_id
            )

            return response

        except Exception as e:
            # Increment failure count
            self._increment_failure_count()

            # Track error type in metrics
            error_type = type(e).__name__
            self.metrics["error_types"][error_type] = self.metrics["error_types"].get(error_type, 0) + 1

            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"query_id": query_id, "query": agent_request.query[:100]}
            )
            log_agent_error(self.logger, error, "process_query", query_id=query_id)
            raise error

        finally:
            # Clean up the thread
            try:
                self.openai_client.beta.threads.delete(thread_id=thread_id)
                self.logger.debug(f"Cleaned up thread: {thread_id}", extra={"query_id": query_id})
            except APIConnectionError as e:
                self.logger.warning(f"OpenAI API connection error cleaning up thread {thread_id}: {str(e)}", extra={
                    "thread_id": thread_id,
                    "query_id": query_id
                })
            except APIError as e:
                self.logger.warning(f"OpenAI API error cleaning up thread {thread_id}: {str(e)}", extra={
                    "thread_id": thread_id,
                    "query_id": query_id,
                    "error_type": type(e).__name__
                })
            except Exception as cleanup_error:
                self.logger.warning(f"Error cleaning up thread {thread_id}: {str(cleanup_error)}", extra={"query_id": query_id})

    def _wait_for_run_completion(self, thread_id: str, run_id: str, timeout: int = 300):
        """
        Wait for a run to complete with a timeout.

        Args:
            thread_id: The ID of the thread
            run_id: The ID of the run
            timeout: Maximum time to wait in seconds

        Returns:
            The completed run object
        """
        import time
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                run = self.openai_client.beta.threads.runs.retrieve(
                    thread_id=thread_id,
                    run_id=run_id
                )

                if run.status in ["completed", "failed", "cancelled", "expired"]:
                    return run

            except APIConnectionError as e:
                self.logger.warning(f"OpenAI API connection error while waiting for run {run_id}: {str(e)}", extra={
                    "thread_id": thread_id,
                    "run_id": run_id,
                    "attempt": "connection_retry"
                })
                time.sleep(2)  # Wait before retrying
                continue
            except RateLimitError as e:
                self.logger.warning(f"OpenAI API rate limit exceeded for run {run_id}: {str(e)}", extra={
                    "thread_id": thread_id,
                    "run_id": run_id,
                    "wait_time": 5
                })
                time.sleep(5)  # Wait before retrying
                continue
            except AuthenticationError as e:
                self.logger.error(f"OpenAI API authentication error for run {run_id}: {str(e)}", extra={
                    "thread_id": thread_id,
                    "run_id": run_id
                })
                raise e
            except APIError as e:
                self.logger.error(f"OpenAI API error while waiting for run {run_id}: {str(e)}", extra={
                    "thread_id": thread_id,
                    "run_id": run_id,
                    "error_type": type(e).__name__
                })
                raise e

            time.sleep(1)

        # If we've reached the timeout, cancel the run
        try:
            self.openai_client.beta.threads.runs.cancel(
                thread_id=thread_id,
                run_id=run_id
            )
        except Exception as cancel_error:
            self.logger.warning(f"Error canceling run {run_id} after timeout: {str(cancel_error)}", extra={
                "thread_id": thread_id,
                "run_id": run_id
            })

        raise TimeoutError(f"Run {run_id} did not complete within {timeout} seconds")

    async def _retrieve_chunks_for_query(self, query: str) -> List[RetrievedChunk]:
        """
        Retrieve relevant chunks from the knowledge base for a query.

        Args:
            query: The query to search for

        Returns:
            List of retrieved chunks with metadata
        """
        import time
        start_time = time.time()

        try:
            # Create a retrieval request
            retrieval_request = RetrievalToolRequest(
                query=query,
                top_k=5,  # Retrieve top 5 chunks
                min_relevance_score=0.3  # Minimum relevance score
            )

            # Use the existing retrieval service
            retrieved_chunks = await self.retrieval_service.retrieve_chunks(
                query=retrieval_request.query,
                top_k=retrieval_request.top_k,
                min_relevance_score=retrieval_request.min_relevance_score
            )

            retrieval_time = time.time() - start_time
            log_agent_retrieval(
                self.logger,
                query,
                len(retrieved_chunks),
                retrieval_time
            )

            return retrieved_chunks

        except Exception as e:
            retrieval_time = time.time() - start_time
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RETRIEVAL_ERROR,
                self.logger,
                {"query": query[:100], "retrieval_time": retrieval_time}
            )
            log_agent_error(self.logger, error, "retrieve_chunks", query_id=query[:10])
            raise error

    def _generate_citations_from_chunks(self, chunks: List[RetrievedChunk]) -> List[SourceCitation]:
        """
        Generate source citations from retrieved chunks.

        Args:
            chunks: List of retrieved chunks

        Returns:
            List of source citations
        """
        citations = []
        for i, chunk in enumerate(chunks):
            # Generate a URL for the citation if possible
            citation_url = self._generate_citation_url(chunk)

            # Extract additional metadata from the chunk
            document_title = None
            section_title = None
            position_in_document = None
            context_before = None
            context_after = None

            if chunk.metadata:
                document_title = chunk.metadata.get('document_title') or chunk.metadata.get('title')
                section_title = chunk.metadata.get('section_title') or chunk.metadata.get('section')
                position_in_document = chunk.metadata.get('position_in_document') or chunk.metadata.get('position')

                # Extract context if available in metadata
                context_before = chunk.metadata.get('context_before')
                context_after = chunk.metadata.get('context_after')

            # Create citation with enhanced formatting using the new model fields
            citation = SourceCitation(
                id=f"citation_{i+1}",
                source_document=chunk.source_document,
                document_title=document_title,
                page_number=chunk.page_number,
                section_title=section_title,
                chunk_text=chunk.content,
                relevance_score=chunk.relevance_score,
                url=citation_url,
                position_in_document=position_in_document,
                context_before=context_before,
                context_after=context_after,
                citation_format="APA",  # Default citation format, could be configurable
                access_date=datetime.now().isoformat()
            )
            citations.append(citation)

        return citations

    def _generate_citation_url(self, chunk: RetrievedChunk) -> Optional[str]:
        """
        Generate a URL for a citation based on chunk metadata.

        Args:
            chunk: The retrieved chunk to generate a URL for

        Returns:
            URL string if possible, None otherwise
        """
        try:
            # Check if we have document metadata to construct a URL
            if chunk.metadata and isinstance(chunk.metadata, dict):
                # Look for common URL-related fields in metadata
                doc_id = chunk.metadata.get('doc_id') or chunk.metadata.get('document_id')
                doc_uuid = chunk.metadata.get('doc_uuid')  # More robust document identifier
                file_path = chunk.metadata.get('file_path')
                source_url = chunk.metadata.get('source_url')
                external_url = chunk.metadata.get('external_url')  # Specific external URL
                document_uri = chunk.metadata.get('document_uri')  # Full document URI

                # Priority 1: Use external URL if provided
                if external_url:
                    return external_url

                # Priority 2: Use source URL if provided
                if source_url:
                    return source_url

                # Priority 3: Use document URI if provided
                if document_uri:
                    return document_uri

                # Priority 4: Construct URL using document UUID (more robust than simple ID)
                if doc_uuid:
                    page_ref = f"#page={chunk.page_number}" if chunk.page_number else ""
                    return f"/documents/{doc_uuid}{page_ref}"

                # Priority 5: Construct URL using document ID and file path
                if doc_id and file_path:
                    # Sanitize file path for URL
                    import urllib.parse
                    safe_path = urllib.parse.quote(file_path.replace(" ", "_").replace("\\", "/"))
                    page_ref = f"#page={chunk.page_number}" if chunk.page_number else ""
                    return f"/documents/{doc_id}/{safe_path}{page_ref}"

                # Priority 6: Construct URL using just document ID
                if doc_id:
                    page_ref = f"#page={chunk.page_number}" if chunk.page_number else ""
                    return f"/documents/{doc_id}{page_ref}"

            # If no specific metadata, try to construct from source document
            if chunk.source_document:
                # Sanitize document name for URL
                import urllib.parse
                doc_name = urllib.parse.quote(chunk.source_document.replace(" ", "_").replace("/", "_").replace("..", ""))
                page_ref = f"#page={chunk.page_number}" if chunk.page_number else ""
                return f"/documents/{doc_name}{page_ref}"

            # If no source document, try to use chunk ID if available in metadata
            if chunk.metadata and isinstance(chunk.metadata, dict):
                chunk_id = chunk.metadata.get('chunk_id') or chunk.metadata.get('id')
                if chunk_id:
                    return f"/chunks/{chunk_id}"

            return None
        except Exception:
            # If there's any error generating the URL, return None
            self.logger.warning(f"Could not generate citation URL for chunk from {chunk.source_document}")
            return None

    def _format_citations_for_response(self, citations: List[SourceCitation], response: str) -> str:
        """
        Format citations to be included in the agent response.

        Args:
            citations: List of citations to format
            response: The agent's response to append citations to

        Returns:
            Formatted response with citations
        """
        if not citations:
            return response

        # Add a citations section to the response
        formatted_citations = "\n\n## Sources and Citations:\n"

        # Sort citations by relevance score (highest first) for better presentation
        sorted_citations = sorted(citations, key=lambda c: c.relevance_score, reverse=True)

        for i, citation in enumerate(sorted_citations, 1):
            # Create citation reference with index
            page_ref = f" (p. {citation.page_number})" if citation.page_number else ""
            url_ref = f" [{citation.source_document}{page_ref}]({citation.url})" if citation.url else f" {citation.source_document}{page_ref}"

            # Include relevance score and additional context if available
            relevance_text = f" (Relevance: {citation.relevance_score:.2f})"
            section_info = f" - Section: {citation.section_title}" if citation.section_title else ""
            doc_title = f" - {citation.document_title}" if citation.document_title else ""

            # Format citation with index for better multiple citation handling
            formatted_citations += f"{i}. {doc_title}{url_ref}{relevance_text}{section_info}\n"

            # Add context if available for complex responses
            if citation.context_before or citation.context_after:
                context_info = ""
                if citation.context_before:
                    context_info += f"  - Context before: {citation.context_before[:100]}{'...' if len(citation.context_before) > 100 else ''}\n"
                if citation.context_after:
                    context_info += f"  - Context after: {citation.context_after[:100]}{'...' if len(citation.context_after) > 100 else ''}\n"
                formatted_citations += context_info

        return response + formatted_citations

    def _calculate_citation_relevance(self, query: str, chunk: RetrievedChunk, agent_response: str = "") -> float:
        """
        Calculate the relevance of a citation to the original query and agent response.

        Args:
            query: The original query
            chunk: The retrieved chunk/citation
            agent_response: The agent's response (for additional context matching)

        Returns:
            Relevance score between 0.0 and 1.0
        """
        try:
            from sklearn.feature_extraction.text import TfidfVectorizer
            from sklearn.metrics.pairwise import cosine_similarity
            import numpy as np

            # Use the original relevance score as the base
            base_score = chunk.relevance_score

            # If we have the agent response, calculate semantic similarity between
            # the response and the chunk content
            response_similarity = 0.0
            if agent_response.strip():
                try:
                    # Simple approach using TF-IDF and cosine similarity
                    vectorizer = TfidfVectorizer()
                    vectors = vectorizer.fit_transform([agent_response, chunk.content])
                    response_similarity = cosine_similarity(vectors[0:1], vectors[1:2])[0][0]
                except:
                    # If TF-IDF fails, use a simple word overlap approach
                    query_words = set(query.lower().split())
                    chunk_words = set(chunk.content.lower().split())
                    overlap = len(query_words.intersection(chunk_words))
                    total_words = len(query_words.union(chunk_words))
                    response_similarity = overlap / total_words if total_words > 0 else 0.0

            # Calculate query-chunk similarity
            query_words = set(query.lower().split())
            chunk_words = set(chunk.content.lower().split())
            query_chunk_similarity = len(query_words.intersection(chunk_words)) / len(query_words) if query_words else 0.0

            # Weighted combination of base score, query similarity, and response similarity
            final_score = (base_score * 0.6) + (query_chunk_similarity * 0.3) + (response_similarity * 0.1)

            # Ensure the score is within bounds
            return min(1.0, max(0.0, final_score))

        except Exception as e:
            self.logger.warning(f"Error calculating citation relevance: {str(e)}")
            # Fall back to the original relevance score
            return chunk.relevance_score

    def _validate_citation_relevance(self, citation: SourceCitation, min_relevance_threshold: float = 0.3) -> bool:
        """
        Validate if a citation meets the minimum relevance threshold.

        Args:
            citation: The citation to validate
            min_relevance_threshold: Minimum relevance score required

        Returns:
            Boolean indicating if the citation is relevant enough
        """
        return citation.relevance_score >= min_relevance_threshold

    def _adjust_citation_scores(self, citations: List[SourceCitation], query: str, agent_response: str = "") -> List[SourceCitation]:
        """
        Adjust citation scores based on additional relevance calculations.

        Args:
            citations: List of citations to adjust scores for
            query: The original query
            agent_response: The agent's response

        Returns:
            List of citations with adjusted relevance scores
        """
        adjusted_citations = []

        for citation in citations:
            # Create a temporary RetrievedChunk object from the citation to pass to the relevance calculation
            # This is a workaround since we don't have the original RetrievedChunk objects here
            temp_chunk = RetrievedChunk(
                id=citation.id,
                content=citation.chunk_text,
                source_document=citation.source_document,
                page_number=citation.page_number,
                relevance_score=citation.relevance_score,
                metadata={
                    "document_title": citation.document_title,
                    "section_title": citation.section_title,
                    "position_in_document": citation.position_in_document
                } if citation.document_title or citation.section_title or citation.position_in_document else {}
            )

            # Calculate enhanced relevance score using the existing method
            adjusted_score = self._calculate_citation_relevance(query, temp_chunk, agent_response)

            # Ensure the score is within bounds
            adjusted_score = min(1.0, max(0.0, adjusted_score))

            # Create a new citation with the adjusted score
            adjusted_citation = SourceCitation(
                id=citation.id,
                source_document=citation.source_document,
                document_title=citation.document_title,
                page_number=citation.page_number,
                section_title=citation.section_title,
                chunk_text=citation.chunk_text,
                relevance_score=adjusted_score,
                url=citation.url,
                position_in_document=citation.position_in_document,
                context_before=citation.context_before,
                context_after=citation.context_after,
                citation_format=citation.citation_format,
                access_date=citation.access_date,
                retrieved_at=citation.retrieved_at
            )
            adjusted_citations.append(adjusted_citation)

        # Sort citations by relevance score in descending order
        adjusted_citations.sort(key=lambda c: c.relevance_score, reverse=True)

        return adjusted_citations

    async def _validate_response_grounding(self, query: str, response: str, retrieved_chunks: List[RetrievedChunk]) -> Dict[str, Any]:
        """
        Validate that a response is properly grounded in the retrieved chunks.

        This is a private method used internally to validate the grounding
        of the agent's response in the retrieved content.

        Args:
            query: The original query
            response: The agent's response to validate
            retrieved_chunks: List of retrieved chunks to validate against

        Returns:
            Dict containing validation results and grounding score
        """
        try:
            # Calculate grounding score based on how well the response aligns with retrieved content
            grounding_score = self._calculate_grounding_score(response, retrieved_chunks)

            # Check if citations are properly linked to retrieved content
            citation_accuracy = self._validate_citation_accuracy(
                [{"chunk_text": chunk.content} for chunk in retrieved_chunks],
                retrieved_chunks
            )

            validation_result = {
                "grounding_score": grounding_score,
                "citation_accuracy": citation_accuracy,
                "is_valid": grounding_score >= 0.6 and citation_accuracy >= 0.6,  # Thresholds can be configured
                "feedback": self._generate_validation_feedback(grounding_score, citation_accuracy),
                "timestamp": datetime.now(),
                "validation_details": {
                    "response_length": len(response),
                    "retrieved_chunks_count": len(retrieved_chunks),
                    "query_similarity": grounding_score  # Approximation
                }
            }

            # Log the validation result
            self.logger.info(f"Response grounding validation completed, score: {grounding_score}", extra={
                "query": query[:50] if len(query) > 50 else query,
                "grounding_score": grounding_score,
                "citation_accuracy": citation_accuracy
            })

            return validation_result

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_GROUNDING_ERROR,
                self.logger,
                {"operation": "validate_response_grounding", "query": query[:100]}
            )
            log_agent_error(self.logger, error, "_validate_response_grounding")
            # Return a default validation result in case of error
            return {
                "grounding_score": 0.0,
                "citation_accuracy": 0.0,
                "is_valid": False,
                "feedback": "Validation failed due to error",
                "timestamp": datetime.now()
            }

    async def validate_response_grounding(self, query: str, response: str, citations: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate that a response is properly grounded in the knowledge base.

        Args:
            query: The original query
            response: The agent's response to validate
            citations: List of citations to validate against

        Returns:
            Dict containing validation results and grounding score
        """
        import uuid
        query_id = str(uuid.uuid4())

        # Additional validation beyond the API level
        if not query or not query.strip():
            raise AgentValidationError("Query cannot be empty or whitespace only")

        if not response or not response.strip():
            raise AgentValidationError("Response cannot be empty or whitespace only")

        try:
            # Retrieve relevant chunks for the query
            retrieved_chunks = await self._retrieve_chunks_for_query(query)

            # Calculate grounding score based on how well the response aligns with retrieved content
            grounding_score = self._calculate_grounding_score(response, retrieved_chunks)

            # Check if citations are properly linked to retrieved content
            citation_accuracy = self._validate_citation_accuracy(citations, retrieved_chunks)

            validation_result = {
                "grounding_score": grounding_score,
                "citation_accuracy": citation_accuracy,
                "is_valid": grounding_score >= 0.6 and citation_accuracy >= 0.6,  # Thresholds can be configured
                "feedback": self._generate_validation_feedback(grounding_score, citation_accuracy),
                "timestamp": datetime.now()
            }

            # Log the validation result
            log_agent_validation(
                self.logger,
                query,
                grounding_score,
                validation_result,
                query_id=query_id
            )

            return validation_result

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_GROUNDING_ERROR,
                self.logger,
                {"query_id": query_id, "query": query[:100]}
            )
            log_agent_error(self.logger, error, "validate_response_grounding", query_id=query_id)
            raise error

    def _calculate_grounding_score(self, response: str, chunks: List[RetrievedChunk]) -> float:
        """
        Calculate how well the response is grounded in the retrieved chunks.

        Args:
            response: The agent's response
            chunks: Retrieved chunks to validate against

        Returns:
            Grounding score between 0.0 and 1.0
        """
        if not chunks:
            return 0.0

        # Simple approach: calculate overlap between response and retrieved content
        response_lower = response.lower()
        total_relevant_content = 0
        total_content = 0

        for chunk in chunks:
            chunk_content = chunk.content.lower()
            total_content += len(chunk_content)

            # Count overlapping words/phrases
            chunk_words = set(chunk_content.split())
            response_words = set(response_lower.split())
            overlap = len(chunk_words.intersection(response_words))
            total_relevant_content += overlap

        if total_content == 0:
            return 0.0

        # Normalize the score
        score = min(1.0, total_relevant_content / (len(response.split()) * len(chunks)))
        return score

    def _validate_citation_accuracy(self, citations: List[Dict[str, Any]], chunks: List[RetrievedChunk]) -> float:
        """
        Validate the accuracy of citations against retrieved chunks.

        Args:
            citations: List of citations from the response
            chunks: Retrieved chunks to validate against

        Returns:
            Citation accuracy score between 0.0 and 1.0
        """
        if not citations or not chunks:
            return 0.0

        # Count how many citations match retrieved chunks
        matching_citations = 0
        for citation in citations:
            for chunk in chunks:
                # Check if citation content matches any chunk content
                if citation.get('chunk_text', '').lower() in chunk.content.lower():
                    matching_citations += 1
                    break

        accuracy = matching_citations / len(citations)
        return accuracy

    def _generate_validation_feedback(self, grounding_score: float, citation_accuracy: float) -> str:
        """
        Generate human-readable feedback for validation results.

        Args:
            grounding_score: The grounding score
            citation_accuracy: The citation accuracy score

        Returns:
            Human-readable feedback string
        """
        if grounding_score < 0.6:
            return "Response may not be sufficiently grounded in the knowledge base. Consider verifying facts against provided sources."
        elif citation_accuracy < 0.6:
            return "Citations may not accurately reference the provided sources. Verify that citations correspond to actual retrieved content."
        else:
            return "Response appears to be well-grounded with accurate citations."

    async def process_feedback(self, query: str, response: str, feedback: str, rating: Optional[int] = None) -> Dict[str, Any]:
        """
        Process user feedback to improve future responses.

        Args:
            query: The original query
            response: The agent's response
            feedback: User's feedback
            rating: Optional numerical rating (1-5)

        Returns:
            Dict confirming feedback processing
        """
        # Additional validation beyond the API level
        if not query or not query.strip():
            raise AgentValidationError("Query cannot be empty or whitespace only")

        if not response or not response.strip():
            raise AgentValidationError("Response cannot be empty or whitespace only")

        if not feedback or not feedback.strip():
            raise AgentValidationError("Feedback cannot be empty or whitespace only")

        if rating is not None and (rating < 1 or rating > 5):
            raise AgentValidationError("Rating must be between 1 and 5")

        self.logger.info(f"Processing feedback for query: {query[:50]}...")

        # In a real implementation, this would store feedback for analysis
        # and potentially model retraining
        feedback_record = {
            "query": query,
            "response": response,
            "feedback": feedback,
            "rating": rating,
            "timestamp": datetime.now()
        }

        # Log feedback for potential future analysis
        self.logger.info(f"Feedback received: {feedback[:100]}...")

        return {
            "status": "received",
            "message": "Thank you for your feedback. This will help improve future responses.",
            "feedback_id": f"feedback_{datetime.now().timestamp()}"
        }

    async def health_check(self) -> bool:
        """
        Perform a health check on the agent service.

        Returns:
            Boolean indicating if the service is healthy
        """
        try:
            # Test OpenAI client connection
            if self.settings.openai_api_key:
                # Make a simple API call to test connectivity
                models = self.openai_client.models.list()
                if not models.data:
                    self.logger.warning("Health check: No OpenAI models available")
                    return False

            # Test retrieval service
            test_retrieval = await self._retrieve_chunks_for_query("health check")

            return True

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"operation": "health_check"}
            )
            log_agent_error(self.logger, error, "health_check")
            return False

    async def create_agent_thread(self, initial_message: str = None) -> str:
        """
        Create a new agent thread for conversation.

        Args:
            initial_message: Optional initial message to add to the thread

        Returns:
            Thread ID
        """
        try:
            thread = self.openai_client.beta.threads.create()
            thread_id = thread.id

            if initial_message:
                self.openai_client.beta.threads.messages.create(
                    thread_id=thread_id,
                    role="user",
                    content=initial_message
                )

            self.logger.info(f"Created new agent thread: {thread_id}", extra={"thread_id": thread_id})
            return thread_id

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"operation": "create_thread"}
            )
            log_agent_error(self.logger, error, "create_agent_thread")
            raise error

    async def run_agent_thread(self, thread_id: str) -> str:
        """
        Run the agent on a specific thread and return the response.

        Args:
            thread_id: The ID of the thread to run

        Returns:
            The agent's response
        """
        try:
            # Run the agent with the configured assistant
            run = self.openai_client.beta.threads.runs.create(
                thread_id=thread_id,
                assistant_id=self.agent_id
            )

            # Wait for completion
            run = self._wait_for_run_completion(thread_id, run.id)

            # Get the messages from the thread
            messages = self.openai_client.beta.threads.messages.list(
                thread_id=thread_id,
                order="asc"
            )

            # Extract the agent's response
            agent_response_content = ""
            for message in messages.data:
                if message.role == "assistant":
                    for content_block in message.content:
                        if content_block.type == "text":
                            agent_response_content += content_block.text.value

            self.logger.info(f"Successfully ran agent thread {thread_id}, response length: {len(agent_response_content)}", extra={"thread_id": thread_id})
            return agent_response_content

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"thread_id": thread_id, "operation": "run_thread"}
            )
            log_agent_error(self.logger, error, "run_agent_thread", thread_id=thread_id)
            raise error

    async def delete_agent_thread(self, thread_id: str) -> bool:
        """
        Delete an agent thread to free up resources.

        Args:
            thread_id: The ID of the thread to delete

        Returns:
            Boolean indicating success
        """
        try:
            self.openai_client.beta.threads.delete(thread_id=thread_id)
            self.logger.info(f"Deleted agent thread: {thread_id}", extra={"thread_id": thread_id})
            return True

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"thread_id": thread_id, "operation": "delete_thread"}
            )
            log_agent_error(self.logger, error, "delete_agent_thread", thread_id=thread_id)
            return False

    async def get_thread_messages(self, thread_id: str) -> list:
        """
        Get all messages from a thread.

        Args:
            thread_id: The ID of the thread

        Returns:
            List of messages in the thread
        """
        try:
            messages = self.openai_client.beta.threads.messages.list(
                thread_id=thread_id,
                order="asc"
            )
            self.logger.debug(f"Retrieved {len(messages.data)} messages from thread {thread_id}", extra={"thread_id": thread_id})
            return messages.data

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"thread_id": thread_id, "operation": "get_messages"}
            )
            log_agent_error(self.logger, error, "get_thread_messages", thread_id=thread_id)
            raise error

    async def add_message_to_thread(self, thread_id: str, role: str, content: str) -> bool:
        """
        Add a message to an existing thread.

        Args:
            thread_id: The ID of the thread
            role: The role of the message ('user' or 'assistant')
            content: The content of the message

        Returns:
            Boolean indicating success
        """
        try:
            self.openai_client.beta.threads.messages.create(
                thread_id=thread_id,
                role=role,
                content=content
            )
            self.logger.debug(f"Added message to thread {thread_id}", extra={"thread_id": thread_id})
            return True

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"thread_id": thread_id, "operation": "add_message"}
            )
            log_agent_error(self.logger, error, "add_message_to_thread", thread_id=thread_id)
            return False