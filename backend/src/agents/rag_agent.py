"""
RAG agent module for the OpenAI RAG Agent feature.

This module contains the core implementation of the RAG Agent using the OpenAI Agents SDK,
including agent orchestration, thread management, and integration with retrieval tools.
"""

import asyncio
import json
import logging
from typing import Dict, Any, List, Optional, Union
from pydantic import BaseModel
import openai
from openai import OpenAI
from openai import APIError, APIConnectionError, RateLimitError, AuthenticationError, NotFoundError, ConflictError, UnprocessableEntityError
from datetime import datetime

from ..models.agent import AgentRequest, AgentResponse, SourceCitation
from ..config import get_settings
from ..utils.agent_logging import get_agent_logger, log_agent_error, log_agent_thread_operation
from ..utils.agent_errors import handle_agent_error, AgentErrorCode
from .retrieval_tool import get_retrieval_tool_manager, RetrievalToolManager


class RAGAgent:
    """
    Core implementation of the RAG Agent using the OpenAI Agents SDK.

    This class handles agent orchestration, thread management, and integration
    with retrieval tools to provide grounded responses based on the knowledge base.
    """

    def __init__(self):
        """Initialize the RAGAgent with required dependencies."""
        self.logger = get_agent_logger(__name__)
        self.settings = get_settings()
        self.openai_client = OpenAI(api_key=self.settings.openai_api_key)

        # Get the retrieval tool manager
        self.retrieval_tool_manager = get_retrieval_tool_manager()

        # Agent configuration
        self.model = self.settings.openai_model or "gpt-4-turbo-preview"
        self.agent_id = self.settings.openai_agent_id

        # Initialize or create an assistant if one doesn't exist
        self.assistant = self._initialize_assistant()

        self.logger.info(f"RAGAgent initialized with assistant ID: {self.assistant.id if self.assistant else 'None'}")

    def _initialize_assistant(self):
        """
        Initialize or create an OpenAI Assistant for the RAG Agent.

        Returns:
            OpenAI Assistant object
        """
        try:
            if self.agent_id:
                # Try to retrieve existing assistant
                try:
                    assistant = self.openai_client.beta.assistants.retrieve(self.agent_id)
                    self.logger.info(f"Retrieved existing assistant: {assistant.id}")
                    return assistant
                except openai.NotFoundError:
                    self.logger.warning(f"Assistant with ID {self.agent_id} not found, creating new one")

            # Log the tool registration
            tool_definitions = self.retrieval_tool_manager.get_tool_definitions()
            self.logger.info(f"Registering {len(tool_definitions)} tools with assistant", extra={
                "tool_count": len(tool_definitions),
                "tool_names": [tool.get("function", {}).get("name") for tool in tool_definitions]
            })

            # Create a new assistant with retrieval tool
            assistant = self.openai_client.beta.assistants.create(
                name="RAG Knowledge Base Assistant",
                description="An assistant that answers questions using information from a knowledge base",
                model=self.model,
                tools=tool_definitions,
                # Add instructions for the assistant
                instructions=(
                    "You are a helpful assistant that answers questions based on information from a knowledge base. "
                    "When a user asks a question, use the retrieval tool to find relevant information from the knowledge base. "
                    "Always provide answers that are grounded in the retrieved information and include proper citations. "
                    "If the retrieved information doesn't contain the answer to the user's question, say so explicitly. "
                    "Do not make up information that is not in the retrieved results."
                )
            )

            self.logger.info(f"Created new assistant: {assistant.id}", extra={"assistant_id": assistant.id})
            return assistant

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"operation": "initialize_assistant"}
            )
            log_agent_error(self.logger, error, "initialize_assistant")
            raise error

    async def create_thread(self, initial_message: Optional[str] = None) -> str:
        """
        Create a new thread for agent interaction.

        Args:
            initial_message: Optional initial message to add to the thread

        Returns:
            Thread ID
        """
        try:
            try:
                thread = self.openai_client.beta.threads.create()
                thread_id = thread.id
            except RateLimitError as e:
                self.logger.warning(f"OpenAI API rate limit exceeded when creating thread: {str(e)}")
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RATE_LIMIT_ERROR,
                    message="Rate limit exceeded. Please try again later.",
                    details={"original_error": str(e)}
                )
            except AuthenticationError as e:
                self.logger.error(f"OpenAI API authentication error when creating thread: {str(e)}")
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_AUTHENTICATION_ERROR,
                    message="Authentication failed. Please check your API key.",
                    details={"original_error": str(e)}
                )
            except APIConnectionError as e:
                self.logger.error(f"OpenAI API connection error when creating thread: {str(e)}")
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_CONNECTION_ERROR,
                    message="Connection error occurred. Please try again.",
                    details={"original_error": str(e)}
                )
            except APIError as e:
                self.logger.error(f"OpenAI API error when creating thread: {str(e)}", extra={
                    "error_type": type(e).__name__
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RUNTIME_ERROR,
                    message=f"Error creating thread: {str(e)}",
                    details={"original_error": str(e)}
                )

            if initial_message:
                try:
                    self.openai_client.beta.threads.messages.create(
                        thread_id=thread_id,
                        role="user",
                        content=initial_message
                    )
                except RateLimitError as e:
                    self.logger.warning(f"OpenAI API rate limit exceeded when adding message to thread: {str(e)}", extra={
                        "thread_id": thread_id
                    })
                    raise AgentError(
                        error_code=AgentErrorCode.AGENT_RATE_LIMIT_ERROR,
                        message="Rate limit exceeded. Please try again later.",
                        details={"original_error": str(e)}
                    )
                except APIError as e:
                    self.logger.error(f"OpenAI API error when adding message to thread: {str(e)}", extra={
                        "thread_id": thread_id,
                        "error_type": type(e).__name__
                    })
                    raise AgentError(
                        error_code=AgentErrorCode.AGENT_RUNTIME_ERROR,
                        message=f"Error adding message to thread: {str(e)}",
                        details={"original_error": str(e)}
                    )

            log_agent_thread_operation(self.logger, "create", thread_id)
            return thread_id

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"operation": "create_thread"}
            )
            log_agent_error(self.logger, error, "create_thread")
            raise error

    async def run_agent(self, thread_id: str, user_input: str) -> str:
        """
        Run the agent on a specific thread with user input.

        Args:
            thread_id: The ID of the thread to run
            user_input: The user's input/query

        Returns:
            The agent's response
        """
        try:
            # Add the user's message to the thread
            try:
                self.openai_client.beta.threads.messages.create(
                    thread_id=thread_id,
                    role="user",
                    content=user_input
                )
            except RateLimitError as e:
                self.logger.warning(f"OpenAI API rate limit exceeded when adding message to thread: {str(e)}", extra={
                    "thread_id": thread_id
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RATE_LIMIT_ERROR,
                    message="Rate limit exceeded. Please try again later.",
                    details={"original_error": str(e)}
                )
            except APIError as e:
                self.logger.error(f"OpenAI API error when adding message to thread: {str(e)}", extra={
                    "thread_id": thread_id,
                    "error_type": type(e).__name__
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RUNTIME_ERROR,
                    message=f"Error adding message to thread: {str(e)}",
                    details={"original_error": str(e)}
                )

            # Run the assistant with tools
            try:
                run = self.openai_client.beta.threads.runs.create(
                    thread_id=thread_id,
                    assistant_id=self.assistant.id,
                    tools=self.retrieval_tool_manager.get_tool_definitions()  # Register tools for this run
                )
            except RateLimitError as e:
                self.logger.warning(f"OpenAI API rate limit exceeded when starting run: {str(e)}", extra={
                    "thread_id": thread_id
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RATE_LIMIT_ERROR,
                    message="Rate limit exceeded. Please try again later.",
                    details={"original_error": str(e)}
                )
            except APIError as e:
                self.logger.error(f"OpenAI API error when starting run: {str(e)}", extra={
                    "thread_id": thread_id,
                    "error_type": type(e).__name__
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RUNTIME_ERROR,
                    message=f"Error starting agent run: {str(e)}",
                    details={"original_error": str(e)}
                )

            # Wait for the run to complete
            completed_run = await self._wait_for_run_completion(thread_id, run.id)

            # Get the messages from the thread
            try:
                messages = self.openai_client.beta.threads.messages.list(
                    thread_id=thread_id,
                    order="asc"
                )
            except APIError as e:
                self.logger.error(f"OpenAI API error when retrieving messages: {str(e)}", extra={
                    "thread_id": thread_id,
                    "error_type": type(e).__name__
                })
                raise AgentError(
                    error_code=AgentErrorCode.AGENT_RUNTIME_ERROR,
                    message=f"Error retrieving messages: {str(e)}",
                    details={"original_error": str(e)}
                )

            # Extract the agent's response
            agent_response_content = ""
            for message in messages.data:
                if message.role == "assistant":
                    for content_block in message.content:
                        if content_block.type == "text":
                            agent_response_content += content_block.text.value

            self.logger.info(f"Completed agent run, response length: {len(agent_response_content)}", extra={"thread_id": thread_id})
            return agent_response_content

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"thread_id": thread_id, "operation": "run_agent"}
            )
            log_agent_error(self.logger, error, "run_agent", thread_id=thread_id)
            raise error

    async def _wait_for_run_completion(self, thread_id: str, run_id: str, timeout: int = 300) -> Any:
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
            run = self.openai_client.beta.threads.runs.retrieve(
                thread_id=thread_id,
                run_id=run_id
            )

            if run.status in ["completed", "failed", "cancelled", "expired"]:
                if run.status == "completed":
                    self.logger.info(f"Run {run_id} completed successfully")
                    return run
                elif run.status == "failed":
                    self.logger.error(f"Run {run_id} failed: {run.last_error}")
                    raise Exception(f"Run failed: {run.last_error}")
                else:
                    self.logger.warning(f"Run {run_id} ended with status: {run.status}")
                    raise Exception(f"Run ended with status: {run.status}")

            # Check if the run requires action (tool calls)
            if run.status == "requires_action":
                await self._handle_tool_calls(thread_id, run.id)

            time.sleep(1)

        # If we've reached the timeout, cancel the run
        try:
            self.openai_client.beta.threads.runs.cancel(
                thread_id=thread_id,
                run_id=run_id
            )
        except Exception as cancel_error:
            self.logger.warning(f"Error canceling run {run_id}: {str(cancel_error)}")

        raise TimeoutError(f"Run {run_id} did not complete within {timeout} seconds")

    async def _handle_tool_calls(self, thread_id: str, run_id: str) -> None:
        """
        Handle tool calls required by the assistant.

        Args:
            thread_id: The ID of the thread
            run_id: The ID of the run
        """
        try:
            # Get the run to check what tool calls are required
            run = self.openai_client.beta.threads.runs.retrieve(
                thread_id=thread_id,
                run_id=run_id
            )

            if run.status == "requires_action" and run.required_action:
                tool_calls = run.required_action.submit_tool_outputs.tool_calls

                tool_outputs = []
                for tool_call in tool_calls:
                    # Extract tool name and arguments
                    tool_name = tool_call.function.name
                    tool_arguments = json.loads(tool_call.function.arguments)

                    # Log the tool call
                    self.logger.info(f"Processing tool call: {tool_name}", extra={
                        "thread_id": thread_id,
                        "run_id": run_id,
                        "tool_call_id": tool_call.id,
                        "tool_name": tool_name
                    })

                    # Execute the appropriate tool
                    if tool_name == "retrieve_knowledge_base":
                        result = await self.retrieval_tool_manager.run_default_tool(tool_arguments)
                        tool_outputs.append({
                            "tool_call_id": tool_call.id,
                            "output": result
                        })

                        # Log successful tool execution
                        result_data = json.loads(result) if result.startswith('{') else {"result": result}
                        self.logger.info(f"Tool call {tool_name} completed", extra={
                            "thread_id": thread_id,
                            "run_id": run_id,
                            "tool_call_id": tool_call.id,
                            "result_length": len(result),
                            "retrieved_chunks": result_data.get("total_chunks", 0) if isinstance(result_data, dict) else 0
                        })
                    else:
                        # For unknown tools, return an error
                        error_output = json.dumps({"error": f"Unknown tool: {tool_name}"})
                        tool_outputs.append({
                            "tool_call_id": tool_call.id,
                            "output": error_output
                        })

                        self.logger.warning(f"Unknown tool called: {tool_name}", extra={
                            "thread_id": thread_id,
                            "run_id": run_id,
                            "tool_call_id": tool_call.id
                        })

                # Submit tool outputs back to the run with error handling
                try:
                    self.openai_client.beta.threads.runs.submit_tool_outputs(
                        thread_id=thread_id,
                        run_id=run_id,
                        tool_outputs=tool_outputs
                    )
                except RateLimitError as e:
                    self.logger.warning(f"OpenAI API rate limit exceeded when submitting tool outputs: {str(e)}", extra={
                        "thread_id": thread_id,
                        "run_id": run_id,
                        "tool_output_count": len(tool_outputs)
                    })
                    raise AgentError(
                        error_code=AgentErrorCode.AGENT_RATE_LIMIT_ERROR,
                        message="Rate limit exceeded when submitting tool outputs.",
                        details={"original_error": str(e)}
                    )
                except APIError as e:
                    self.logger.error(f"OpenAI API error when submitting tool outputs: {str(e)}", extra={
                        "thread_id": thread_id,
                        "run_id": run_id,
                        "tool_output_count": len(tool_outputs),
                        "error_type": type(e).__name__
                    })
                    raise AgentError(
                        error_code=AgentErrorCode.AGENT_RUNTIME_ERROR,
                        message=f"Error submitting tool outputs: {str(e)}",
                        details={"original_error": str(e)}
                    )

                self.logger.info(f"Submitted {len(tool_outputs)} tool outputs", extra={
                    "thread_id": thread_id,
                    "run_id": run_id,
                    "tool_output_count": len(tool_outputs)
                })

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"thread_id": thread_id, "run_id": run_id, "operation": "handle_tool_calls"}
            )
            log_agent_error(self.logger, error, "handle_tool_calls", thread_id=thread_id)
            raise error

    async def process_query(self, agent_request: AgentRequest) -> AgentResponse:
        """
        Process an agent query end-to-end and return a grounded response.

        This method creates a thread, runs the agent, and formats the response
        with proper citations and metadata.

        Args:
            agent_request: The query and parameters from the user

        Returns:
            AgentResponse: The agent's response with citations and metadata
        """
        import uuid
        query_id = str(uuid.uuid4())
        start_time = datetime.now()

        log_agent_thread_operation(self.logger, "create", f"query_{query_id}")

        # Create a new thread for this query
        thread_id = await self.create_thread()

        try:
            # Run the agent with the user's query
            agent_response_content = await self.run_agent(thread_id, agent_request.query)

            # For now, we'll return a basic response; in a more advanced implementation,
            # we would extract citations from the agent's response
            response = AgentResponse(
                answer=agent_response_content,
                citations=[],  # Will be populated based on retrieved content
                query=agent_request.query,
                tokens_used=len(agent_response_content.split()),  # Approximate token count
                confidence=0.8  # Default confidence - should be calculated based on grounding
            )

            response_time = (datetime.now() - start_time).total_seconds()
            self.logger.info(f"Successfully processed agent query, response length: {len(agent_response_content)}", extra={
                "query_id": query_id,
                "thread_id": thread_id,
                "response_time": response_time,
                "response_length": len(agent_response_content)
            })
            return response

        except Exception as e:
            error = handle_agent_error(
                e,
                AgentErrorCode.AGENT_RUNTIME_ERROR,
                self.logger,
                {"query_id": query_id, "thread_id": thread_id, "query": agent_request.query[:100]}
            )
            log_agent_error(self.logger, error, "process_query", query_id=query_id, thread_id=thread_id)
            raise error

        finally:
            # Clean up the thread
            try:
                self.openai_client.beta.threads.delete(thread_id=thread_id)
                log_agent_thread_operation(self.logger, "delete", thread_id, query_id=query_id)
            except Exception as cleanup_error:
                self.logger.warning(f"Error cleaning up thread {thread_id}: {str(cleanup_error)}", extra={
                    "query_id": query_id,
                    "thread_id": thread_id
                })

    async def get_agent_status(self) -> Dict[str, Any]:
        """
        Get the status of the RAG Agent.

        Returns:
            Dictionary containing agent status information
        """
        try:
            status = {
                "assistant_id": self.assistant.id if self.assistant else None,
                "model": self.model,
                "is_available": self.assistant is not None,
                "timestamp": datetime.now(),
                "retrieval_tools_available": len(self.retrieval_tool_manager.get_tool_definitions())
            }

            return status

        except Exception as e:
            self.logger.error(f"Error getting agent status: {str(e)}", exc_info=True)
            return {
                "assistant_id": None,
                "model": self.model,
                "is_available": False,
                "timestamp": datetime.now(),
                "error": str(e)
            }

    async def update_assistant_instructions(self, new_instructions: str) -> bool:
        """
        Update the assistant's instructions.

        Args:
            new_instructions: New instructions for the assistant

        Returns:
            Boolean indicating success
        """
        try:
            if not self.assistant:
                raise Exception("No assistant available to update")

            # Update the assistant with new instructions
            self.assistant = self.openai_client.beta.assistants.update(
                self.assistant.id,
                instructions=new_instructions
            )

            self.logger.info("Successfully updated assistant instructions")
            return True

        except Exception as e:
            self.logger.error(f"Error updating assistant instructions: {str(e)}", exc_info=True)
            return False

    async def delete_assistant(self) -> bool:
        """
        Delete the current assistant (if needed for cleanup).

        Returns:
            Boolean indicating success
        """
        try:
            if not self.assistant:
                self.logger.warning("No assistant to delete")
                return True

            # Delete the assistant
            self.openai_client.beta.assistants.delete(self.assistant.id)
            self.logger.info(f"Deleted assistant: {self.assistant.id}")

            # Clear reference
            self.assistant = None
            return True

        except Exception as e:
            self.logger.error(f"Error deleting assistant: {str(e)}", exc_info=True)
            return False


class RAGAgentManager:
    """
    Manager class to handle multiple RAG agents.

    This class can manage multiple RAG agents if needed for different purposes
    or configurations.
    """

    def __init__(self):
        """Initialize the RAGAgentManager."""
        self.logger = get_agent_logger(__name__)
        self.agents: Dict[str, RAGAgent] = {}

        # Create a default agent
        self._default_agent = RAGAgent()
        self.agents["default"] = self._default_agent

        self.logger.info("RAGAgentManager initialized successfully")

    def get_default_agent(self) -> RAGAgent:
        """
        Get the default RAG agent.

        Returns:
            Default RAGAgent instance
        """
        return self._default_agent

    def get_agent_by_name(self, name: str) -> Optional[RAGAgent]:
        """
        Get a specific agent by name.

        Args:
            name: Name of the agent to retrieve

        Returns:
            RAGAgent instance if found, None otherwise
        """
        return self.agents.get(name)

    async def process_query_with_default_agent(self, agent_request: AgentRequest) -> AgentResponse:
        """
        Process a query using the default agent.

        Args:
            agent_request: The query and parameters from the user

        Returns:
            AgentResponse: The agent's response with citations and metadata
        """
        return await self._default_agent.process_query(agent_request)


# Global instance for easy access
rag_agent_manager = RAGAgentManager()


def get_rag_agent() -> RAGAgent:
    """
    Get the default RAG agent instance.

    Returns:
        Default RAGAgent instance
    """
    return rag_agent_manager.get_default_agent()


def get_rag_agent_manager() -> RAGAgentManager:
    """
    Get the RAG agent manager instance.

    Returns:
        RAGAgentManager instance
    """
    return rag_agent_manager


