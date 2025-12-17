"""
API endpoints for the validation functionality of RAG Retrieval Validation feature.

This module provides endpoints for running validation tests and measuring metrics.
"""

import asyncio
import time
import logging
from typing import Optional, List, Dict, Any
from fastapi import APIRouter, HTTPException, status
from fastapi.responses import JSONResponse

from ..models.retrieval import ValidationBenchmark, ValidationResponse, RetrievalResponse
from ..models.validation import ValidationResult, MetricResult
from ..services.validation_service import get_validation_service
from ..utils.errors import handle_validation_error
from ..utils.logging import log_validation_request, log_validation_response

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post("/validation/run", response_model=ValidationResponse, status_code=status.HTTP_200_OK)
async def run_validation(benchmark: ValidationBenchmark):
    """
    Run retrieval validation benchmark.
    
    Executes a set of validation queries and computes evaluation metrics.
    
    Args:
        benchmark: ValidationBenchmark object with test parameters
        
    Returns:
        ValidationResponse with computed metrics
    """
    try:
        logger.info(f"Starting validation: {benchmark.benchmark_name}")
        log_validation_request(benchmark.benchmark_name, len(benchmark.question_set))
        
        # Get validation service instance
        validation_svc = get_validation_service()
        
        # Execute the validation
        start_time = time.time()
        result = await validation_svc.run_validation(benchmark)
        total_time = time.time() - start_time
        
        # Convert to ValidationResponse
        validation_response = ValidationResponse(
            benchmark_name=benchmark.benchmark_name,
            results=result.retrieval_results,
            metrics=result.metrics,
            validation_latency=total_time
        )
        
        log_validation_response(
            benchmark_name=benchmark.benchmark_name,
            metrics=result.metrics,
            validation_latency=total_time
        )
        
        logger.info(f"Validation completed in {total_time:.4f}s")
        return validation_response
        
    except Exception as e:
        logger.error(f"Error in run_validation: {str(e)}")
        return handle_validation_error(e, benchmark.benchmark_name)


@router.post("/validation/metrics", response_model=ValidationResult, status_code=status.HTTP_200_OK)
async def calculate_metrics(
    benchmark: ValidationBenchmark,
    retrieval_results: List[RetrievalResponse]
):
    """
    Calculate validation metrics for a set of retrieval results.
    
    Args:
        benchmark: ValidationBenchmark with ground truth mappings
        retrieval_results: List of retrieval results to evaluate
        
    Returns:
        ValidationResult with calculated metrics
    """
    try:
        logger.info(f"Calculating metrics for {len(retrieval_results)} results")
        
        # Get validation service instance
        validation_svc = get_validation_service()
        
        # Calculate metrics
        start_time = time.time()
        result = await validation_svc.calculate_metrics(benchmark, retrieval_results)
        total_time = time.time() - start_time
        
        logger.info(f"Metrics calculated in {total_time:.4f}s")
        return result
        
    except Exception as e:
        logger.error(f"Error in calculate_metrics: {str(e)}")
        return handle_validation_error(e, benchmark.benchmark_name)