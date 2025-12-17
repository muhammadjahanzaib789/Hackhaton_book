"""
Retrieval tool module for the OpenAI RAG Agent feature.

This module contains the implementation of the retrieval tool that the OpenAI Agent
can use to access the knowledge base and retrieve relevant information.
"""

import json
import logging
from typing import Dict, Any, List, Optional
from pydantic import BaseModel, Field
import openai
from openai import OpenAI
import jsonschema

from ..models.agent import RetrievedChunk, RetrievalToolRequest, RetrievalToolResponse
from ..services.retrieval_service import RetrievalService
from ..config import get_settings
from ..utils.agent_logging import get_agent_logger, log_agent_error
from ..utils.agent_errors import handle_agent_error, AgentErrorCode


class RetrievalTool:
    """
    Implementation of the retrieval tool for the OpenAI Agent.

    This tool allows the OpenAI Agent to retrieve relevant information from
    the knowledge base when answering questions. It integrates with the
    existing retrieval pipeline to provide grounded responses.
    """

    def __init__(self):
        """Initialize the RetrievalTool with required dependencies."""
        self.logger = get_agent_logger(__name__)
        self.settings = get_settings()
        self.retrieval_service = RetrievalService()

        # Initialize OpenAI client for tool integration
        self.openai_client = OpenAI(api_key=self.settings.openai_api_key)

        # Initialize metrics tracking
        self.metrics = {
            "total_invocations": 0,
            "successful_invocations": 0,
            "failed_invocations": 0,
            "total_retrieval_time": 0.0,
            "avg_retrieval_time": 0.0,
            "total_chunks_retrieved": 0,
            "avg_chunks_per_invocation": 0,
            "avg_relevance_score": 0.0,
            "invocation_count_by_hour": {},
            "error_count_by_type": {}
        }

        self.logger.info("RetrievalTool initialized successfully")

    def _update_invocation_metrics(self, retrieval_time: float, chunks_retrieved: int,
                                 avg_relevance: float = 0.0, success: bool = True) -> None:
        """
        Update metrics for a tool invocation.

        Args:
            retrieval_time: Time taken for the retrieval in seconds
            chunks_retrieved: Number of chunks retrieved
            avg_relevance: Average relevance score of retrieved chunks
            success: Whether the invocation was successful
        """
        try:
            import time
            from collections import defaultdict

            # Update counters
            self.metrics["total_invocations"] += 1

            if success:
                self.metrics["successful_invocations"] += 1
                self.metrics["total_retrieval_time"] += retrieval_time
                self.metrics["total_chunks_retrieved"] += chunks_retrieved

                # Update hourly invocation count
                current_hour = int(time.time() // 3600)
                self.metrics["invocation_count_by_hour"][current_hour] = self.metrics["invocation_count_by_hour"].get(current_hour, 0) + 1

                # Update average calculations
                total_successful = self.metrics["successful_invocations"]
                self.metrics["avg_retrieval_time"] = self.metrics["total_retrieval_time"] / total_successful
                self.metrics["avg_chunks_per_invocation"] = self.metrics["total_chunks_retrieved"] / total_successful

                if avg_relevance > 0:
                    current_avg = self.metrics["avg_relevance_score"]
                    self.metrics["avg_relevance_score"] = ((current_avg * (total_successful - 1)) + avg_relevance) / total_successful
            else:
                self.metrics["failed_invocations"] += 1

            self.logger.debug("Tool invocation metrics updated", extra={
                "total_invocations": self.metrics["total_invocations"],
                "successful_invocations": self.metrics["successful_invocations"],
                "avg_retrieval_time": round(self.metrics["avg_retrieval_time"], 3)
            })
        except Exception as e:
            self.logger.error(f"Error updating tool invocation metrics: {str(e)}", exc_info=True)

    def _record_error_type(self, error_type: str) -> None:
        """
        Record an error type for metrics tracking.

        Args:
            error_type: Type of error that occurred
        """
        try:
            self.metrics["error_count_by_type"][error_type] = self.metrics["error_count_by_type"].get(error_type, 0) + 1
        except Exception as e:
            self.logger.error(f"Error recording error type: {str(e)}", exc_info=True)

    async def get_tool_invocation_metrics(self) -> Dict[str, Any]:
        """
        Get current tool invocation metrics.

        Returns:
            Dictionary containing tool invocation metrics
        """
        try:
            import time

            # Clean up old hourly data (keep only last 24 hours)
            current_hour = int(time.time() // 3600)
            cutoff_hour = current_hour - 24
            self.metrics["invocation_count_by_hour"] = {
                hour: count for hour, count in self.metrics["invocation_count_by_hour"].items()
                if hour > cutoff_hour
            }

            # Calculate invocations per hour
            recent_invocations = sum(self.metrics["invocation_count_by_hour"].values())
            invocations_per_hour = recent_invocations / min(24, len(self.metrics["invocation_count_by_hour"])) if self.metrics["invocation_count_by_hour"] else 0

            total_invocations = self.metrics["total_invocations"]
            success_rate = (self.metrics["successful_invocations"] / total_invocations * 100) if total_invocations > 0 else 0
            failure_rate = (self.metrics["failed_invocations"] / total_invocations * 100) if total_invocations > 0 else 0

            metrics_report = {
                "summary": {
                    "total_invocations": total_invocations,
                    "successful_invocations": self.metrics["successful_invocations"],
                    "failed_invocations": self.metrics["failed_invocations"],
                    "success_rate": round(success_rate, 2),
                    "failure_rate": round(failure_rate, 2),
                    "invocations_per_hour": round(invocations_per_hour, 2)
                },
                "timing": {
                    "avg_retrieval_time": round(self.metrics["avg_retrieval_time"], 3),
                    "total_retrieval_time": round(self.metrics["total_retrieval_time"], 3)
                },
                "content_metrics": {
                    "total_chunks_retrieved": self.metrics["total_chunks_retrieved"],
                    "avg_chunks_per_invocation": round(self.metrics["avg_chunks_per_invocation"], 2),
                    "avg_relevance_score": round(self.metrics["avg_relevance_score"], 3)
                },
                "error_metrics": {
                    "error_count_by_type": self.metrics["error_count_by_type"]
                },
                "timestamp": __import__('datetime').datetime.now().isoformat()
            }

            self.logger.info("Tool invocation metrics retrieved", extra={"metrics": metrics_report["summary"]})
            return metrics_report

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"operation": "get_tool_invocation_metrics"}
            )
            log_agent_error(self.logger, error, "get_tool_invocation_metrics")
            raise error

    def get_tool_definition(self) -> Dict[str, Any]:
        """
        Get the OpenAI-compatible tool definition for this retrieval tool.

        Returns:
            Dict containing the tool definition that can be registered with OpenAI
        """
        return {
            "type": "function",
            "function": {
                "name": "retrieve_knowledge_base",
                "description": "Retrieve relevant information from the knowledge base to answer questions. This function searches the knowledge base using semantic similarity to find the most relevant chunks of information related to the query.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The search query to find relevant information in the knowledge base. Should be a clear, specific question or statement related to the information you're looking for.",
                            "minLength": 1,
                            "maxLength": 500
                        },
                        "top_k": {
                            "type": "integer",
                            "description": "Number of top results to retrieve (default: 5). Must be between 1 and 20.",
                            "default": 5,
                            "minimum": 1,
                            "maximum": 20
                        },
                        "min_relevance_score": {
                            "type": "number",
                            "description": "Minimum relevance score for results (default: 0.3). Must be between 0.0 and 1.0. Higher values mean more relevant results.",
                            "default": 0.3,
                            "minimum": 0.0,
                            "maximum": 1.0
                        }
                    },
                    "required": ["query"],
                    "additionalProperties": False
                }
            }
        }

    async def run_retrieval(self, tool_input: Dict[str, Any]) -> str:
        """
        Execute the retrieval tool with the given input.

        This method is called by the OpenAI Agent when it decides to use the
        retrieval tool. It processes the input query and returns formatted
        results that the agent can use to answer questions.

        Args:
            tool_input: Dictionary containing the tool input parameters
                       Expected keys: query (required), top_k (optional), min_relevance_score (optional)

        Returns:
            JSON string containing the retrieval results formatted for the agent
        """
        import time
        import uuid
        tool_call_id = str(uuid.uuid4())
        start_time = time.time()

        self.logger.info(f"Running retrieval tool for query: {tool_input.get('query', '')[:50]}...", extra={
            "tool_call_id": tool_call_id
        })

        try:
            # Validate input parameters based on tool definition
            validation_result = await self.validate_tool_input(tool_input)
            if not validation_result:
                raise ValueError(f"Invalid tool input: {tool_input}")

            # Extract parameters from tool input
            query = tool_input.get("query")
            top_k = tool_input.get("top_k", 5)
            min_relevance_score = tool_input.get("min_relevance_score", 0.3)

            if not query:
                raise ValueError("Query is required for retrieval tool")

            # Validate parameters with more specific constraints from tool definition
            if not isinstance(top_k, int) or top_k < 1 or top_k > 20:
                top_k = 5  # Default value
            if not isinstance(min_relevance_score, (int, float)) or min_relevance_score < 0.0 or min_relevance_score > 1.0:
                min_relevance_score = 0.3  # Default value

            # Create retrieval request
            retrieval_request = RetrievalToolRequest(
                query=query,
                top_k=top_k,
                min_relevance_score=min_relevance_score
            )

            # Perform retrieval using the existing retrieval service
            retrieved_chunks = await self.retrieval_service.retrieve_chunks(
                query=retrieval_request.query,
                top_k=retrieval_request.top_k,
                min_relevance_score=retrieval_request.min_relevance_score
            )

            # Calculate metrics for successful retrieval
            retrieval_time = time.time() - start_time
            avg_relevance = sum(chunk.relevance_score for chunk in retrieved_chunks) / len(retrieved_chunks) if retrieved_chunks else 0

            # Update metrics for successful invocation
            self._update_invocation_metrics(
                retrieval_time=retrieval_time,
                chunks_retrieved=len(retrieved_chunks),
                avg_relevance=avg_relevance,
                success=True
            )

            # Format results for the agent
            formatted_results = self._format_results_for_agent(retrieved_chunks, query)

            self.logger.info(f"Retrieval completed, found {len(retrieved_chunks)} chunks", extra={
                "tool_call_id": tool_call_id,
                "query": query[:50] if len(query) > 50 else query,
                "chunks_found": len(retrieved_chunks),
                "retrieval_time": round(retrieval_time, 3),
                "top_k_requested": top_k,
                "avg_relevance": round(avg_relevance, 3)
            })
            return json.dumps(formatted_results)

        except Exception as e:
            # Calculate time even for failed retrieval
            retrieval_time = time.time() - start_time

            # Update metrics for failed invocation
            self._update_invocation_metrics(
                retrieval_time=retrieval_time,
                chunks_retrieved=0,
                success=False
            )

            # Record the error type
            error_type = type(e).__name__
            self._record_error_type(error_type)

            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RETRIEVAL_ERROR,
                self.logger,
                {"tool_call_id": tool_call_id, "query": tool_input.get("query", "")[:100], "retrieval_time": retrieval_time}
            )
            log_agent_error(self.logger, error, "run_retrieval", query_id=tool_call_id[:10])

            # Return structured error response for the agent
            error_result = {
                "error": {
                    "type": "retrieval_error",
                    "message": str(error.message),
                    "code": error.error_code.value,
                    "query": tool_input.get("query", ""),
                    "timestamp": error.timestamp.isoformat()
                }
            }
            return json.dumps(error_result)

    def _format_results_for_agent(self, chunks: List[RetrievedChunk], query: str) -> Dict[str, Any]:
        """
        Format retrieval results in a way that's useful for the OpenAI Agent.

        Args:
            chunks: List of retrieved chunks
            query: Original query that triggered the retrieval

        Returns:
            Dictionary containing formatted results
        """
        formatted_chunks = []
        for chunk in chunks:
            # Create a formatted chunk with agent-friendly structure
            formatted_chunk = {
                "id": chunk.id,
                "content": chunk.content,
                "source_document": chunk.source_document,
                "page_number": chunk.page_number,
                "relevance_score": round(float(chunk.relevance_score), 3),  # Round for cleaner display
                "metadata": chunk.metadata or {},
                # Additional fields that are helpful for the agent
                "summary": self._extract_content_summary(chunk.content),
                "key_terms": self._extract_key_terms(chunk.content),
                "confidence": round(float(chunk.relevance_score), 3)  # Alias for relevance_score
            }
            formatted_chunks.append(formatted_chunk)

        # Calculate aggregate metrics
        avg_relevance = sum(chunk.relevance_score for chunk in chunks) / len(chunks) if chunks else 0
        total_chars = sum(len(chunk.content) for chunk in chunks)

        result = {
            "query": query,
            "retrieved_chunks": formatted_chunks,
            "total_chunks": len(formatted_chunks),
            "aggregate_metrics": {
                "avg_relevance_score": round(float(avg_relevance), 3),
                "total_characters": total_chars,
                "query_relevance": round(float(avg_relevance), 3)  # How relevant the results are to the query
            },
            "timestamp": __import__('datetime').datetime.now().isoformat(),
            "response_format_version": "1.0"
        }

        return result

    def _extract_content_summary(self, content: str, max_length: int = 100) -> str:
        """
        Extract a summary from the content for quick reference.

        Args:
            content: The content to summarize
            max_length: Maximum length of the summary

        Returns:
            Summary string
        """
        if len(content) <= max_length:
            return content

        # Truncate content and add ellipsis
        truncated = content[:max_length].rsplit(' ', 1)[0]  # Avoid cutting in the middle of a word
        return truncated + "..."

    def _extract_key_terms(self, content: str, max_terms: int = 5) -> List[str]:
        """
        Extract key terms from the content that might be relevant to the agent.

        Args:
            content: The content to extract terms from
            max_terms: Maximum number of terms to extract

        Returns:
            List of key terms
        """
        import re

        # Simple approach: extract capitalized words, numbers, and important terms
        # This could be enhanced with NLP techniques in the future
        words = re.findall(r'\b[A-Z][a-z]+\b|\b[A-Z]{2,}\b|\b\d+\w*\b|\b\w+ing\b|\b\w+ed\b', content)

        # Remove duplicates while preserving order and limit to max_terms
        unique_words = []
        seen = set()
        for word in words:
            word_lower = word.lower()
            if word_lower not in seen and len(unique_words) < max_terms:
                unique_words.append(word)
                seen.add(word_lower)

        return unique_words

    def get_tool_json_schema(self) -> Dict[str, Any]:
        """
        Get the JSON schema for the retrieval tool input.

        Returns:
            Dict containing the JSON schema for validation
        """
        return {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The search query to find relevant information in the knowledge base. Should be a clear, specific question or statement related to the information you're looking for.",
                    "minLength": 1,
                    "maxLength": 500
                },
                "top_k": {
                    "type": "integer",
                    "description": "Number of top results to retrieve (default: 5). Must be between 1 and 20.",
                    "default": 5,
                    "minimum": 1,
                    "maximum": 20
                },
                "min_relevance_score": {
                    "type": "number",
                    "description": "Minimum relevance score for results (default: 0.3). Must be between 0.0 and 1.0. Higher values mean more relevant results.",
                    "default": 0.3,
                    "minimum": 0.0,
                    "maximum": 1.0
                }
            },
            "required": ["query"],
            "additionalProperties": False
        }

    async def validate_tool_input(self, tool_input: Dict[str, Any]) -> bool:
        """
        Validate the input for the retrieval tool using JSON schema validation.

        Args:
            tool_input: Dictionary containing the tool input parameters

        Returns:
            Boolean indicating if the input is valid
        """
        try:
            # Get the schema for validation
            schema = self.get_tool_json_schema()

            # Use jsonschema to validate the input
            jsonschema.validate(tool_input, schema)

            # Additional custom validation beyond JSON schema
            query = tool_input.get("query")
            if query and len(query.strip()) == 0:
                self.logger.warning("Retrieval tool query cannot be empty or whitespace")
                return False

            # If we get here, validation passed
            self.logger.debug(f"Retrieval tool input validation passed for query: {query[:50] if query else 'N/A'}")
            return True

        except jsonschema.ValidationError as e:
            self.logger.warning(f"Retrieval tool input validation failed: {str(e)}")
            return False
        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_VALIDATION_ERROR,
                self.logger,
                {"operation": "validate_tool_input", "tool_input_keys": list(tool_input.keys())}
            )
            log_agent_error(self.logger, error, "validate_tool_input")
            return False

    async def get_retrieval_stats(self) -> Dict[str, Any]:
        """
        Get statistics about retrieval performance.

        Returns:
            Dictionary containing retrieval statistics
        """
        try:
            # This would connect to actual stats if available
            # For now, return placeholder stats
            stats = {
                "total_retrievals": 0,  # Would track actual usage
                "avg_response_time": 0.0,  # Would track actual performance
                "avg_chunks_per_query": 0,  # Would track actual usage
                "success_rate": 1.0,  # Would track actual success rate
                "timestamp": __import__('datetime').datetime.now().isoformat()
            }
            return stats

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"operation": "get_retrieval_stats"}
            )
            log_agent_error(self.logger, error, "get_retrieval_stats")
            return {"error": str(error.message), "timestamp": __import__('datetime').datetime.now().isoformat()}


class RetrievalToolManager:
    """
    Manager class to handle multiple retrieval tools and their registration.

    This class can manage multiple retrieval tools if needed and handle
    their registration with the OpenAI Agent system.
    """

    def __init__(self):
        """Initialize the RetrievalToolManager."""
        self.logger = get_agent_logger(__name__)
        self.tools: Dict[str, RetrievalTool] = {}
        self._default_tool = RetrievalTool()
        self.tools["default"] = self._default_tool

        self.logger.info("RetrievalToolManager initialized successfully")

    def get_default_tool(self) -> RetrievalTool:
        """
        Get the default retrieval tool.

        Returns:
            Default RetrievalTool instance
        """
        return self._default_tool

    def get_tool_definitions(self) -> List[Dict[str, Any]]:
        """
        Get tool definitions for all registered tools.

        Returns:
            List of tool definitions that can be registered with OpenAI
        """
        return [tool.get_tool_definition() for tool in self.tools.values()]

    def get_tool_by_name(self, name: str) -> Optional[RetrievalTool]:
        """
        Get a specific tool by name.

        Args:
            name: Name of the tool to retrieve

        Returns:
            RetrievalTool instance if found, None otherwise
        """
        return self.tools.get(name)

    async def run_tool_by_name(self, name: str, tool_input: Dict[str, Any]) -> str:
        """
        Run a specific tool by name.

        Args:
            name: Name of the tool to run
            tool_input: Input parameters for the tool

        Returns:
            Tool result as JSON string
        """
        tool = self.tools.get(name)
        if not tool:
            raise ValueError(f"Tool '{name}' not found")

        return await tool.run_retrieval(tool_input)

    async def run_default_tool(self, tool_input: Dict[str, Any]) -> str:
        """
        Run the default retrieval tool.

        Args:
            tool_input: Input parameters for the tool

        Returns:
            Tool result as JSON string
        """
        return await self._default_tool.run_retrieval(tool_input)


# Global instance for easy access
retrieval_tool_manager = RetrievalToolManager()


def get_retrieval_tool() -> RetrievalTool:
    """
    Get the default retrieval tool instance.

    Returns:
        Default RetrievalTool instance
    """
    return retrieval_tool_manager.get_default_tool()


def get_retrieval_tool_manager() -> RetrievalToolManager:
    """
    Get the retrieval tool manager instance.

    Returns:
        RetrievalToolManager instance
    """
    return retrieval_tool_manager