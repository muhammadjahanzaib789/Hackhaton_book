"""
Validation service for RAG Retrieval Validation feature.

This module provides functionality for running validation tests
and computing evaluation metrics like precision, recall, and MRR.
"""

import asyncio
import time
import logging
from typing import List, Dict, Any, Tuple, Optional
from pydantic import BaseModel

from ..models.retrieval import ValidationBenchmark, ValidationResponse, Query, RetrievalResponse
from ..models.validation import ValidationResult, MetricResult
from ..services.retrieval_service import get_retrieval_service
from ..utils.errors import ValidationError

logger = logging.getLogger(__name__)


class ValidationService:
    """Service for running validation tests and computing metrics."""
    
    def __init__(self):
        self.retrieval_service = get_retrieval_service()
    
    async def run_validation(self, benchmark: ValidationBenchmark) -> ValidationResult:
        """
        Run a complete validation benchmark.
        
        Args:
            benchmark: ValidationBenchmark with test parameters
            
        Returns:
            ValidationResult with computed metrics
        """
        start_time = time.time()
        successful_queries = 0
        failed_queries = 0
        retrieval_results = []
        
        try:
            # Execute each query in the benchmark
            for question in benchmark.question_set:
                try:
                    # Create a query for this question
                    query = Query(
                        query_text=question,
                        top_k=10,  # Use 10 for evaluation to have enough results for metrics
                        similarity_threshold=0.0  # Include all results for metric calculation
                    )
                    
                    # Execute the retrieval
                    result = await self.retrieval_service.retrieve(query)
                    retrieval_results.append(result)
                    successful_queries += 1
                    
                except Exception as e:
                    logger.error(f"Failed to process query '{question}': {str(e)}")
                    failed_queries += 1
                    
                    # Create an error response for failed queries
                    error_response = RetrievalResponse(
                        query_id=f"error-{int(time.time())}",
                        results=[],
                        query_latency=0,
                        embedding_generation_time=0,
                        qdrant_search_time=0,
                        post_processing_time=0,
                        status="error",
                        error_message=str(e)
                    )
                    retrieval_results.append(error_response)
        
            # Calculate metrics
            total_queries = len(benchmark.question_set)
            metrics = await self._calculate_validation_metrics(
                benchmark, retrieval_results
            )
            
            # Prepare the final result
            final_result = ValidationResult(
                benchmark_name=benchmark.benchmark_name,
                total_queries=total_queries,
                successful_queries=successful_queries,
                failed_queries=failed_queries,
                metrics=metrics,
                execution_time=time.time() - start_time,
                timestamp=time.time()
            )
            
            return final_result
            
        except Exception as e:
            raise ValidationError(
                message=f"Failed to run validation: {str(e)}",
                error_code="VALIDATION_ERROR",
                details={"benchmark_name": benchmark.benchmark_name}
            )
    
    async def calculate_metrics(
        self, 
        benchmark: ValidationBenchmark, 
        retrieval_results: List[RetrievalResponse]
    ) -> ValidationResult:
        """
        Calculate validation metrics for provided retrieval results.
        
        Args:
            benchmark: ValidationBenchmark with ground truth mappings
            retrieval_results: List of retrieval results to evaluate
            
        Returns:
            ValidationResult with calculated metrics
        """
        start_time = time.time()
        metrics = await self._calculate_validation_metrics(benchmark, retrieval_results)
        
        successful_queries = sum(1 for r in retrieval_results if r.status != "error")
        failed_queries = len(retrieval_results) - successful_queries
        
        result = ValidationResult(
            benchmark_name=benchmark.benchmark_name,
            total_queries=len(retrieval_results),
            successful_queries=successful_queries,
            failed_queries=failed_queries,
            metrics=metrics,
            execution_time=time.time() - start_time,
            timestamp=time.time()
        )
        
        return result
    
    async def _calculate_validation_metrics(
        self,
        benchmark: ValidationBenchmark,
        retrieval_results: List[RetrievalResponse]
    ) -> List[MetricResult]:
        """
        Calculate various validation metrics based on retrieval results.
        
        Args:
            benchmark: ValidationBenchmark with ground truth
            retrieval_results: List of retrieval results to evaluate
            
        Returns:
            List of MetricResult with calculated metrics
        """
        metrics = []
        
        # Calculate metrics based on the requested metric types
        for metric_name in benchmark.metrics:
            if metric_name.startswith('precision@'):
                k = int(metric_name.split('@')[1])
                value = await self._calculate_precision_at_k(benchmark, retrieval_results, k)
                metrics.append(MetricResult(
                    name=metric_name,
                    value=value,
                    description=f"Precision at {k} (relevant retrieved / total retrieved)"
                ))
            
            elif metric_name.startswith('recall@'):
                k = int(metric_name.split('@')[1])
                value = await self._calculate_recall_at_k(benchmark, retrieval_results, k)
                metrics.append(MetricResult(
                    name=metric_name,
                    value=value,
                    description=f"Recall at {k} (relevant retrieved / total relevant)"
                ))
            
            elif metric_name == 'MRR':
                value = await self._calculate_mrr(benchmark, retrieval_results)
                metrics.append(MetricResult(
                    name=metric_name,
                    value=value,
                    description="Mean Reciprocal Rank"
                ))
        
        return metrics
    
    async def _calculate_precision_at_k(
        self,
        benchmark: ValidationBenchmark,
        retrieval_results: List[RetrievalResponse],
        k: int
    ) -> float:
        """
        Calculate precision at k (P@k) for the retrieval results.
        
        Args:
            benchmark: ValidationBenchmark with ground truth
            retrieval_results: List of retrieval results
            k: Number of top results to consider
            
        Returns:
            Precision at k value
        """
        if not retrieval_results:
            return 0.0
        
        total_precision = 0.0
        valid_queries = 0
        
        for i, result in enumerate(retrieval_results):
            if result.status == "error":
                continue  # Skip failed queries
            
            question = benchmark.question_set[i] if i < len(benchmark.question_set) else None
            if not question or question not in benchmark.ground_truth_mappings:
                continue  # Skip if no ground truth for this query
            
            # Get top-k results
            top_k_results = result.results[:k] if len(result.results) >= k else result.results
            
            if not top_k_results:
                total_precision += 0.0
                continue
            
            # Get ground truth relevant IDs for this question
            ground_truth_ids = set(benchmark.ground_truth_mappings[question])
            
            # Count how many of the top-k results are relevant
            relevant_retrieved = sum(
                1 for r in top_k_results if r.chunk_id in ground_truth_ids
            )
            
            # Calculate precision
            precision = relevant_retrieved / len(top_k_results)
            total_precision += precision
            valid_queries += 1
        
        return total_precision / valid_queries if valid_queries > 0 else 0.0
    
    async def _calculate_recall_at_k(
        self,
        benchmark: ValidationBenchmark,
        retrieval_results: List[RetrievalResponse],
        k: int
    ) -> float:
        """
        Calculate recall at k (R@k) for the retrieval results.
        
        Args:
            benchmark: ValidationBenchmark with ground truth
            retrieval_results: List of retrieval results
            k: Number of top results to consider
            
        Returns:
            Recall at k value
        """
        if not retrieval_results:
            return 0.0
        
        total_recall = 0.0
        valid_queries = 0
        
        for i, result in enumerate(retrieval_results):
            if result.status == "error":
                continue  # Skip failed queries
            
            question = benchmark.question_set[i] if i < len(benchmark.question_set) else None
            if not question or question not in benchmark.ground_truth_mappings:
                continue  # Skip if no ground truth for this query
            
            # Get top-k results
            top_k_results = result.results[:k] if len(result.results) >= k else result.results
            
            # Get ground truth relevant IDs for this question
            ground_truth_ids = set(benchmark.ground_truth_mappings[question])
            
            if not ground_truth_ids:
                continue  # Skip if no ground truth relevant items
            
            # Count how many of the ground truth items were retrieved in top-k
            relevant_retrieved = sum(
                1 for r in top_k_results if r.chunk_id in ground_truth_ids
            )
            
            # Calculate recall
            recall = relevant_retrieved / len(ground_truth_ids)
            total_recall += recall
            valid_queries += 1
        
        return total_recall / valid_queries if valid_queries > 0 else 0.0
    
    async def _calculate_mrr(
        self,
        benchmark: ValidationBenchmark,
        retrieval_results: List[RetrievalResponse]
    ) -> float:
        """
        Calculate Mean Reciprocal Rank (MRR) for the retrieval results.
        
        Args:
            benchmark: ValidationBenchmark with ground truth
            retrieval_results: List of retrieval results
            
        Returns:
            MRR value
        """
        if not retrieval_results:
            return 0.0
        
        total_rr = 0.0
        valid_queries = 0
        
        for i, result in enumerate(retrieval_results):
            if result.status == "error":
                continue  # Skip failed queries
            
            question = benchmark.question_set[i] if i < len(benchmark.question_set) else None
            if not question or question not in benchmark.ground_truth_mappings:
                continue  # Skip if no ground truth for this query
            
            # Get ground truth relevant IDs for this question
            ground_truth_ids = set(benchmark.ground_truth_mappings[question])
            
            if not ground_truth_ids:
                continue  # Skip if no ground truth relevant items
            
            # Find the rank of the first relevant item
            first_relevant_rank = None
            for rank, retrieval_result in enumerate(result.results, start=1):
                if retrieval_result.chunk_id in ground_truth_ids:
                    first_relevant_rank = rank
                    break
            
            if first_relevant_rank is not None:
                # Reciprocal rank is 1 divided by the rank of the first relevant result
                rr = 1.0 / first_relevant_rank
            else:
                # If no relevant result found, reciprocal rank is 0
                rr = 0.0
            
            total_rr += rr
            valid_queries += 1
        
        return total_rr / valid_queries if valid_queries > 0 else 0.0


# Global instance
validation_service: Optional[ValidationService] = None


def get_validation_service() -> ValidationService:
    """Get the global validation service instance."""
    global validation_service
    if validation_service is None:
        validation_service = ValidationService()
    return validation_service