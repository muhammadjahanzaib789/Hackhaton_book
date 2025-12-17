"""
Agent API module for the OpenAI RAG Agent feature.

This module contains FastAPI endpoints for the RAG Agent system,
allowing external clients to interact with the agent functionality.
"""

from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from typing import Dict, Any, Optional
import logging
from datetime import datetime

from ..models.agent import AgentRequest, AgentResponse, ErrorResponse, ValidateResponseRequest, FeedbackRequest
from ..services.agent_service import AgentService
from ..config import get_settings
from ..utils.agent_logging import get_agent_logger, log_agent_query_start, log_agent_query_end, log_agent_error
# Rate limiting
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from ..utils.agent_errors import handle_agent_error, AgentErrorCode

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Initialize router
router = APIRouter(prefix="/agent", tags=["agent"])
router._rate_limit_exceeded_handler = _rate_limit_exceeded_handler

# Initialize logger
logger = get_agent_logger(__name__)

# Initialize agent service
agent_service = AgentService()


@router.post("/query",
             response_model=AgentResponse,
             summary="Query the RAG Agent",
             description="Send a query to the RAG Agent and receive a grounded response with citations.",
             responses={
                 200: {
                     "description": "Successfully processed query and returned grounded response",
                     "content": {
                         "application/json": {
                             "example": {
                                 "answer": "The RAG Agent provides grounded responses based on retrieved information...",
                                 "citations": [
                                     {
                                         "id": "citation_1",
                                         "source_document": "physical_ai_book.pdf",
                                         "chunk_text": "Retrieved text chunk...",
                                         "relevance_score": 0.85,
                                         "url": "/documents/physical_ai_book.pdf#page=15"
                                     }
                                 ],
                                 "query": "What is embodied cognition?",
                                 "timestamp": "2024-01-01T10:00:00",
                                 "tokens_used": 150,
                                 "confidence": 0.85
                             }
                         }
                     }
                 },
                 422: {"description": "Validation error in request parameters"},
                 429: {"description": "Rate limit exceeded"},
                 500: {"description": "Internal server error during query processing"}
             })
@limiter.limit("10/minute")  # Limit to 10 queries per minute per IP
async def query_agent(agent_request: AgentRequest) -> AgentResponse:
    """
    Query the RAG Agent with a question and receive a grounded response.

    This endpoint processes the user's query through the RAG Agent, which retrieves
    relevant information from the knowledge base and generates a response based
    on that information with proper citations.

    Args:
        agent_request: The query and parameters for the agent

    Returns:
        AgentResponse: The agent's response with citations and metadata

    Raises:
        HTTPException: If there's an error processing the request
    """
    import uuid
    query_id = str(uuid.uuid4())
    start_time = datetime.now()

    log_agent_query_start(
        logger,
        agent_request.query,
        query_id
    )

    try:
        # Process the query through the agent service
        response = await agent_service.process_query(agent_request)

        response_time = (datetime.now() - start_time).total_seconds()
        log_agent_query_end(
            logger,
            query_id,
            len(response.answer),
            response_time,
            tokens_used=response.tokens_used,
            retrieved_chunks=len(response.citations)
        )

        return response

    except Exception as e:
        error = handle_agent_error(
            e,
            AgentErrorCode.AGENT_RUNTIME_ERROR,
            logger,
            {"query_id": query_id, "query": agent_request.query[:100]}
        )
        log_agent_error(logger, error, "query_agent", query_id=query_id)

        raise HTTPException(
            status_code=500,
            detail=f"Error processing agent query: {str(error.message)}"
        )


@router.post("/validate",
             response_model=Dict[str, Any],
             summary="Validate agent response grounding",
             description="Validate that an agent response is properly grounded in the knowledge base.",
             responses={
                 200: {
                     "description": "Successfully validated agent response grounding",
                     "content": {
                         "application/json": {
                             "example": {
                                 "grounding_score": 0.85,
                                 "citation_accuracy": 0.9,
                                 "is_valid": True,
                                 "feedback": "Response appears to be well-grounded with accurate citations.",
                                 "timestamp": "2024-01-01T10:00:00"
                             }
                         }
                     }
                 },
                 422: {"description": "Validation error in request parameters"},
                 429: {"description": "Rate limit exceeded"},
                 500: {"description": "Internal server error during validation"}
             })
@limiter.limit("20/minute")  # Limit to 20 validations per minute per IP
async def validate_agent_response(validate_request: ValidateResponseRequest) -> Dict[str, Any]:
    """
    Validate that an agent response is properly grounded in the knowledge base.

    This endpoint checks if the provided response is properly grounded in the
    retrieved information from the knowledge base, ensuring the response is
    factually accurate and properly cited.

    Args:
        validate_request: The validation request containing query, response, and citations

    Returns:
        Dict: Validation results with grounding score and feedback
    """
    import uuid
    query_id = str(uuid.uuid4())

    # Log the incoming validation request
    logger.info(f"Validation request received for query: {validate_request.query[:50]}...", extra={
        "query_id": query_id,
        "query_length": len(validate_request.query),
        "response_length": len(validate_request.response),
        "citations_count": len(validate_request.citations) if validate_request.citations else 0
    })

    try:
        # Validate the grounding of the response
        validation_result = await agent_service.validate_response_grounding(
            query=validate_request.query,
            response=validate_request.response,
            citations=validate_request.citations or []
        )

        # Log successful validation
        logger.info(f"Validation completed successfully", extra={
            "query_id": query_id,
            "grounding_score": validation_result.get("grounding_score"),
            "citation_accuracy": validation_result.get("citation_accuracy"),
            "is_valid": validation_result.get("is_valid")
        })

        return validation_result

    except Exception as e:
        error = handle_agent_error(
            e,
            AgentErrorCode.AGENT_GROUNDING_ERROR,
            logger,
            {"query_id": query_id, "query": validate_request.query[:100]}
        )
        log_agent_error(logger, error, "validate_agent_response", query_id=query_id)

        raise HTTPException(
            status_code=500,
            detail=f"Error validating agent response: {str(error.message)}"
        )


@router.get("/health",
            response_model=Dict[str, Any],
            summary="Agent service health check",
            description="Check the health status of the agent service.",
            responses={
                200: {
                    "description": "Agent service is healthy",
                    "content": {
                        "application/json": {
                            "example": {
                                "status": "healthy",
                                "timestamp": "2024-01-01T10:00:00",
                                "service": "rag-agent",
                                "health_check_id": "abc123"
                            }
                        }
                    }
                },
                429: {"description": "Rate limit exceeded"},
                500: {"description": "Agent service is unhealthy"}
            })
@limiter.limit("30/minute")  # Limit to 30 health checks per minute per IP
async def agent_health_check() -> Dict[str, Any]:
    """
    Health check endpoint for the agent service.

    This endpoint verifies that the agent service is operational and
    can process requests.

    Returns:
        Dict: Health status information
    """
    try:
        # Check if the agent service is operational
        is_healthy = await agent_service.health_check()

        health_status = {
            "status": "healthy" if is_healthy else "unhealthy",
            "timestamp": datetime.now(),
            "service": "rag-agent"
        }

        return health_status

    except Exception as e:
        error = handle_agent_error(
            e,
            AgentErrorCode.AGENT_RUNTIME_ERROR,
            logger,
            {"operation": "health_check"}
        )
        log_agent_error(logger, error, "agent_health_check")

        raise HTTPException(
            status_code=500,
            detail=f"Agent health check failed: {str(error.message)}"
        )


@router.post("/feedback",
             response_model=Dict[str, Any],
             summary="Submit feedback on agent response",
             description="Submit feedback to improve the agent's future responses.",
             responses={
                 200: {
                     "description": "Successfully processed feedback",
                     "content": {
                         "application/json": {
                             "example": {
                                 "status": "received",
                                 "message": "Thank you for your feedback. This will help improve future responses.",
                                 "feedback_id": "feedback_1234567890"
                             }
                         }
                     }
                 },
                 422: {"description": "Validation error in request parameters"},
                 429: {"description": "Rate limit exceeded"},
                 500: {"description": "Internal server error during feedback processing"}
             })
@limiter.limit("5/minute")  # Limit to 5 feedback submissions per minute per IP
async def submit_agent_feedback(feedback_request: FeedbackRequest) -> Dict[str, Any]:
    """
    Submit feedback on an agent response to improve future responses.

    This endpoint allows users to provide feedback on the quality and accuracy
    of agent responses, which can be used to improve the system over time.

    Args:
        feedback_request: The feedback request containing query, response, feedback, and rating

    Returns:
        Dict: Confirmation of feedback submission
    """
    import uuid
    query_id = str(uuid.uuid4())

    # Log the incoming feedback request
    logger.info(f"Feedback request received for query: {feedback_request.query[:50]}...", extra={
        "query_id": query_id,
        "query_length": len(feedback_request.query),
        "response_length": len(feedback_request.response),
        "feedback_length": len(feedback_request.feedback),
        "rating": feedback_request.rating
    })

    try:
        # Process the feedback
        feedback_result = await agent_service.process_feedback(
            query=feedback_request.query,
            response=feedback_request.response,
            feedback=feedback_request.feedback,
            rating=feedback_request.rating
        )

        # Log successful feedback processing
        logger.info(f"Feedback processed successfully", extra={
            "query_id": query_id,
            "status": feedback_result.get("status"),
            "feedback_id": feedback_result.get("feedback_id")
        })

        return feedback_result

    except Exception as e:
        error = handle_agent_error(
            e,
            AgentErrorCode.AGENT_RUNTIME_ERROR,
            logger,
            {"query_id": query_id, "query": feedback_request.query[:100]}
        )
        log_agent_error(logger, error, "submit_agent_feedback", query_id=query_id)

        raise HTTPException(
            status_code=500,
            detail=f"Error processing agent feedback: {str(error.message)}"
        )


# Additional utility endpoints can be added here as needed