"""
Information Retrieval Evaluation Metrics Implementation

Provides practical implementations of standard IR metrics:
- Precision@K, Recall@K (order-agnostic)
- MRR@K, MAP@K (order-aware)
- NDCG@K (graded relevance, rank-aware)

All metrics are aligned with TREC and NDCG standards for RAG evaluation.
"""

import numpy as np
import logging
from typing import List, Dict, Set, Optional, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime

logger = logging.getLogger(__name__)


# ============================================================================
# Data Models
# ============================================================================


@dataclass
class MetricScore:
    """Single metric score with metadata."""
    metric_name: str
    value: float
    k: Optional[int] = None
    query_id: Optional[str] = None
    timestamp: Optional[str] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class QueryEvaluationResult:
    """Evaluation result for a single query."""
    query_id: str
    retrieved_doc_ids: List[str]
    relevant_doc_ids: Set[str]
    relevance_scores: Optional[List[int]] = None  # For NDCG
    metrics: Optional[Dict[str, float]] = None
    timestamp: Optional[str] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            "query_id": self.query_id,
            "retrieved_doc_ids": self.retrieved_doc_ids,
            "relevant_doc_ids": list(self.relevant_doc_ids),
            "metrics": self.metrics or {},
            "timestamp": self.timestamp
        }


# ============================================================================
# Order-Agnostic Metrics
# ============================================================================


def precision_at_k(retrieved_ids: List[str], relevant_ids: Set[str], k: int = 5) -> float:
    """
    Calculate Precision@K.

    Precision measures what fraction of the top-k retrieved results are relevant.

    Formula: |relevant ∩ retrieved_k| / k

    Args:
        retrieved_ids: List of retrieved document IDs in rank order
        relevant_ids: Set of relevant document IDs (ground truth)
        k: Cutoff position (default 5)

    Returns:
        float: Precision@K in range [0, 1]

    Examples:
        >>> precision_at_k(['d1', 'd3', 'd5', 'd7', 'd9'], {'d1', 'd5', 'd10'}, k=5)
        0.4  # 2 relevant out of 5
    """
    if k <= 0 or not retrieved_ids:
        return 0.0

    top_k = set(retrieved_ids[:k])
    relevant_in_k = top_k.intersection(relevant_ids)
    return len(relevant_in_k) / k


def recall_at_k(retrieved_ids: List[str], relevant_ids: Set[str], k: int = 5) -> float:
    """
    Calculate Recall@K.

    Recall measures what fraction of all relevant documents appear in top-k.

    Formula: |relevant ∩ retrieved_k| / |relevant|

    Args:
        retrieved_ids: List of retrieved document IDs in rank order
        relevant_ids: Set of relevant document IDs (ground truth)
        k: Cutoff position (default 5)

    Returns:
        float: Recall@K in range [0, 1]

    Examples:
        >>> recall_at_k(['d1', 'd3', 'd5', 'd7', 'd9'], {'d1', 'd5', 'd10'}, k=5)
        0.667  # 2 out of 3 relevant docs found
    """
    if not relevant_ids:
        return 0.0

    if k <= 0 or not retrieved_ids:
        return 0.0

    top_k = set(retrieved_ids[:k])
    relevant_in_k = top_k.intersection(relevant_ids)
    return len(relevant_in_k) / len(relevant_ids)


def f1_at_k(retrieved_ids: List[str], relevant_ids: Set[str], k: int = 5) -> float:
    """
    Calculate F1@K.

    Harmonic mean of Precision@K and Recall@K.

    Formula: 2 * (Precision@K * Recall@K) / (Precision@K + Recall@K)

    Args:
        retrieved_ids: List of retrieved document IDs in rank order
        relevant_ids: Set of relevant document IDs (ground truth)
        k: Cutoff position (default 5)

    Returns:
        float: F1@K in range [0, 1]
    """
    prec = precision_at_k(retrieved_ids, relevant_ids, k)
    rec = recall_at_k(retrieved_ids, relevant_ids, k)

    if prec + rec == 0:
        return 0.0

    return 2 * (prec * rec) / (prec + rec)


def hit_rate_at_k(retrieved_ids: List[str], relevant_ids: Set[str], k: int = 5) -> float:
    """
    Calculate Hit Rate@K (binary indicator of retrieval success).

    Returns 1.0 if at least one relevant document in top-k, else 0.0.

    Args:
        retrieved_ids: List of retrieved document IDs in rank order
        relevant_ids: Set of relevant document IDs (ground truth)
        k: Cutoff position (default 5)

    Returns:
        float: 1.0 if hit, 0.0 otherwise
    """
    if not retrieved_ids or not relevant_ids:
        return 0.0

    top_k = set(retrieved_ids[:k])
    return 1.0 if top_k.intersection(relevant_ids) else 0.0


# ============================================================================
# Order-Aware Metrics
# ============================================================================


def mrr_at_k(retrieved_ids: List[str], relevant_ids: Set[str], k: int = 10) -> float:
    """
    Calculate Mean Reciprocal Rank@K.

    MRR focuses on the rank position of the first relevant result.
    Returns 1/rank where rank is position of first relevant document (1-indexed).
    Returns 0 if no relevant document found.

    Formula: 1 / rank_of_first_relevant (or 0 if none)

    Args:
        retrieved_ids: List of retrieved document IDs in rank order
        relevant_ids: Set of relevant document IDs (ground truth)
        k: Cutoff position (default 10)

    Returns:
        float: MRR@K in range [0, 1]

    Examples:
        >>> mrr_at_k(['d2', 'd4', 'd1', 'd7'], {'d1', 'd5'}, k=10)
        0.333  # First relevant 'd1' is at rank 3 (1-indexed), so 1/3
    """
    if not retrieved_ids or not relevant_ids:
        return 0.0

    for i, doc_id in enumerate(retrieved_ids[:k], start=1):
        if doc_id in relevant_ids:
            return 1.0 / i

    return 0.0


def average_precision_at_k(
    retrieved_ids: List[str],
    relevant_ids: Set[str],
    k: Optional[int] = None
) -> float:
    """
    Calculate Average Precision@K for a single query.

    AP sums precision values at each rank where a relevant document appears,
    then divides by total number of relevant documents.

    Assumes binary relevance (either relevant or not).

    Formula:
        AP = sum(Precision@i for i where doc_i is relevant) / total_relevant

    Args:
        retrieved_ids: List of retrieved document IDs in rank order
        relevant_ids: Set of relevant document IDs (ground truth)
        k: Optional cutoff position (default: use all)

    Returns:
        float: Average Precision in range [0, 1]

    Examples:
        >>> # Results: [Relevant, Non-relevant, Non-relevant, Relevant, Non-relevant]
        >>> average_precision_at_k(['d1', 'd2', 'd3', 'd4', 'd5'], {'d1', 'd4', 'd6'})
        0.5
        # Precision@1 = 1/1 = 1.0 (d1 relevant)
        # Precision@4 = 2/4 = 0.5 (d4 relevant)
        # AP = (1.0 + 0.5) / 3 = 0.5
    """
    if not relevant_ids:
        return 0.0

    if not retrieved_ids:
        return 0.0

    retrieved_subset = retrieved_ids[:k] if k else retrieved_ids
    relevant_count = 0
    precision_sum = 0.0

    for i, doc_id in enumerate(retrieved_subset, start=1):
        if doc_id in relevant_ids:
            relevant_count += 1
            precision_at_i = relevant_count / i
            precision_sum += precision_at_i

    return precision_sum / len(relevant_ids)


# ============================================================================
# Graded Relevance Metrics
# ============================================================================


def dcg_at_k(relevance_scores: List[int], k: int = 10) -> float:
    """
    Calculate Discounted Cumulative Gain@K.

    DCG rewards relevant documents, with stronger discount for lower positions.
    Handles graded relevance (relevance_scores can be 0, 1, 2, 3...).

    Formula: sum(rel_i / log2(i+1)) for i in 1..k

    Args:
        relevance_scores: List of relevance scores (0=not relevant, 1-3=varying relevance)
        k: Cutoff position (default 10)

    Returns:
        float: DCG score (unbounded, used for NDCG normalization)

    Examples:
        >>> dcg_at_k([3, 0, 2, 1, 0], k=5)
        5.43  # 3/log2(2) + 0/log2(3) + 2/log2(4) + 1/log2(5)
    """
    if not relevance_scores:
        return 0.0

    dcg = 0.0
    for i, relevance in enumerate(relevance_scores[:k], start=1):
        discount = np.log2(i + 1)
        dcg += relevance / discount

    return dcg


def ndcg_at_k(relevance_scores: List[int], k: int = 10) -> float:
    """
    Calculate Normalized Discounted Cumulative Gain@K.

    NDCG is the primary metric for RAG evaluation. It normalizes DCG by the
    ideal DCG (perfect ranking), producing a score in [0, 1].

    Handles graded relevance and rewards both presence and position of relevant docs.

    Formula: NDCG@K = DCG@K / IDCG@K

    Where:
        DCG@K = sum(rel_i / log2(i+1))
        IDCG@K = DCG of ideal ranking (sorted descending)

    Args:
        relevance_scores: List of relevance scores (0=not relevant, 1-3=varying relevance)
        k: Cutoff position (default 10)

    Returns:
        float: NDCG@K in range [0, 1]. 1.0 = perfect ranking.

    Examples:
        >>> # Perfect ranking: [3, 2, 1]
        >>> ndcg_at_k([3, 2, 1], k=3)
        1.0
        >>> # Suboptimal ranking: [1, 2, 3]
        >>> ndcg_at_k([1, 2, 3], k=3)
        0.63
    """
    if not relevance_scores:
        return 0.0

    # Calculate DCG
    dcg = dcg_at_k(relevance_scores, k)

    # Calculate Ideal DCG (sorted in descending order)
    ideal_scores = sorted(relevance_scores, reverse=True)[:k]
    idcg = dcg_at_k(ideal_scores, k)

    # Normalize
    if idcg == 0:
        return 0.0

    return dcg / idcg


# ============================================================================
# Batch Aggregation (Mean metrics across multiple queries)
# ============================================================================


def mean_precision_at_k(
    all_queries: List[Tuple[List[str], Set[str]]],
    k: int = 5
) -> float:
    """
    Calculate Mean Precision@K across multiple queries.

    Args:
        all_queries: List of (retrieved_ids, relevant_ids) tuples
        k: Cutoff position

    Returns:
        float: Mean Precision@K
    """
    if not all_queries:
        return 0.0

    scores = [precision_at_k(retr, rel, k) for retr, rel in all_queries]
    return sum(scores) / len(scores)


def mean_recall_at_k(
    all_queries: List[Tuple[List[str], Set[str]]],
    k: int = 5
) -> float:
    """Calculate Mean Recall@K across multiple queries."""
    if not all_queries:
        return 0.0

    scores = [recall_at_k(retr, rel, k) for retr, rel in all_queries]
    return sum(scores) / len(scores)


def mean_mrr_at_k(
    all_queries: List[Tuple[List[str], Set[str]]],
    k: int = 10
) -> float:
    """Calculate Mean MRR@K across multiple queries."""
    if not all_queries:
        return 0.0

    scores = [mrr_at_k(retr, rel, k) for retr, rel in all_queries]
    return sum(scores) / len(scores)


def mean_average_precision_at_k(
    all_queries: List[Tuple[List[str], Set[str]]],
    k: Optional[int] = None
) -> float:
    """Calculate Mean Average Precision@K across multiple queries."""
    if not all_queries:
        return 0.0

    scores = [average_precision_at_k(retr, rel, k) for retr, rel in all_queries]
    return sum(scores) / len(scores)


def mean_ndcg_at_k(all_queries: List[List[int]], k: int = 10) -> float:
    """
    Calculate Mean NDCG@K across multiple queries.

    Args:
        all_queries: List of relevance score lists (each in retrieval order)
        k: Cutoff position

    Returns:
        float: Mean NDCG@K
    """
    if not all_queries:
        return 0.0

    scores = [ndcg_at_k(scores, k) for scores in all_queries]
    return sum(scores) / len(scores)


# ============================================================================
# Batch Evaluation Engine
# ============================================================================


class BatchEvaluator:
    """
    Main evaluation engine for computing multiple metrics across queries.

    This class provides a complete evaluation harness for RAG systems.
    """

    def __init__(self, k_values: List[int] = None):
        """
        Initialize evaluator.

        Args:
            k_values: Cutoff positions to evaluate (default [5, 10])
        """
        self.k_values = k_values or [5, 10]
        self.results = []

    def evaluate_query(
        self,
        query_id: str,
        retrieved_ids: List[str],
        relevant_ids: Set[str],
        relevance_scores: Optional[List[int]] = None,
        metrics: Optional[List[str]] = None
    ) -> QueryEvaluationResult:
        """
        Evaluate a single query against ground truth.

        Args:
            query_id: Unique query identifier
            retrieved_ids: Retrieved documents in rank order
            relevant_ids: Set of relevant document IDs
            relevance_scores: Optional relevance scores for NDCG (0-3 scale)
            metrics: Metrics to compute (default: all)

        Returns:
            QueryEvaluationResult with computed metrics
        """
        if metrics is None:
            metrics = ['precision', 'recall', 'mrr', 'map', 'ndcg']

        query_metrics = {}

        # Compute order-agnostic metrics
        if 'precision' in metrics:
            for k in self.k_values:
                score = precision_at_k(retrieved_ids, relevant_ids, k)
                query_metrics[f"precision@{k}"] = score

        if 'recall' in metrics:
            for k in self.k_values:
                score = recall_at_k(retrieved_ids, relevant_ids, k)
                query_metrics[f"recall@{k}"] = score

        if 'f1' in metrics:
            for k in self.k_values:
                score = f1_at_k(retrieved_ids, relevant_ids, k)
                query_metrics[f"f1@{k}"] = score

        if 'hit_rate' in metrics:
            for k in self.k_values:
                score = hit_rate_at_k(retrieved_ids, relevant_ids, k)
                query_metrics[f"hit_rate@{k}"] = score

        # Compute order-aware metrics
        if 'mrr' in metrics:
            for k in self.k_values:
                score = mrr_at_k(retrieved_ids, relevant_ids, k)
                query_metrics[f"mrr@{k}"] = score

        if 'map' in metrics:
            for k in self.k_values:
                score = average_precision_at_k(retrieved_ids, relevant_ids, k)
                query_metrics[f"map@{k}"] = score

        # Compute graded metrics if relevance scores provided
        if 'ndcg' in metrics and relevance_scores:
            for k in self.k_values:
                score = ndcg_at_k(relevance_scores, k)
                query_metrics[f"ndcg@{k}"] = score

        result = QueryEvaluationResult(
            query_id=query_id,
            retrieved_doc_ids=retrieved_ids,
            relevant_doc_ids=relevant_ids,
            relevance_scores=relevance_scores,
            metrics=query_metrics,
            timestamp=datetime.now().isoformat()
        )

        self.results.append(result)
        return result

    def get_aggregate_metrics(self, metric_names: Optional[List[str]] = None) -> Dict[str, float]:
        """
        Compute aggregate metrics across all evaluated queries.

        Args:
            metric_names: Specific metrics to aggregate (e.g., ['ndcg@10', 'mrr@10'])
                         Default: all

        Returns:
            Dict of aggregated metrics
        """
        if not self.results:
            logger.warning("No results to aggregate")
            return {}

        aggregate = {}

        # Extract all metric names if not specified
        if metric_names is None:
            metric_names = set()
            for result in self.results:
                if result.metrics:
                    metric_names.update(result.metrics.keys())

        # Aggregate each metric
        for metric_name in metric_names:
            scores = []
            for result in self.results:
                if result.metrics and metric_name in result.metrics:
                    scores.append(result.metrics[metric_name])

            if scores:
                aggregate[metric_name] = sum(scores) / len(scores)

        return aggregate

    def get_results_json(self) -> List[Dict]:
        """Get all results as JSON-serializable dicts."""
        return [r.to_dict() for r in self.results]


# ============================================================================
# Utility Functions
# ============================================================================


def convert_binary_to_graded(binary_relevant_ids: Set[str], k: int) -> List[int]:
    """
    Convert binary relevance judgments to graded scores for NDCG computation.

    Args:
        binary_relevant_ids: Set of relevant document IDs
        k: Number of documents to generate scores for

    Returns:
        List of graded relevance scores (1 if relevant, 0 otherwise)
    """
    scores = []
    # This is a placeholder - in practice, you'd have graded judgments
    # This just converts binary to graded (1 for relevant, 0 for not)
    for doc_id in range(k):
        scores.append(1 if str(doc_id) in binary_relevant_ids else 0)
    return scores


def detect_metric_anomalies(
    current_metrics: Dict[str, float],
    previous_metrics: Dict[str, float],
    threshold: float = 0.05
) -> Dict[str, float]:
    """
    Detect significant changes in metrics (regression or improvement).

    Args:
        current_metrics: Current metric values
        previous_metrics: Previous metric values
        threshold: Relative change threshold (default 5%)

    Returns:
        Dict of metrics with significant changes (key: metric, value: pct_change)
    """
    anomalies = {}

    for metric_name in current_metrics:
        if metric_name not in previous_metrics:
            continue

        current = current_metrics[metric_name]
        previous = previous_metrics[metric_name]

        if previous != 0:
            pct_change = (current - previous) / previous
            if abs(pct_change) > threshold:
                anomalies[metric_name] = pct_change

    return anomalies


if __name__ == "__main__":
    # Example usage
    logger.basicConfig(level=logging.INFO)

    # Create evaluator
    evaluator = BatchEvaluator(k_values=[5, 10])

    # Evaluate some queries
    evaluator.evaluate_query(
        query_id="q1",
        retrieved_ids=["d1", "d3", "d5", "d7", "d9"],
        relevant_ids={"d1", "d5", "d10"},
        relevance_scores=[1, 0, 1, 0, 0],
        metrics=['precision', 'recall', 'mrr', 'map', 'ndcg']
    )

    evaluator.evaluate_query(
        query_id="q2",
        retrieved_ids=["d2", "d4", "d1"],
        relevant_ids={"d1", "d2"},
        relevance_scores=[1, 0, 1],
        metrics=['precision', 'recall', 'mrr', 'map', 'ndcg']
    )

    # Get aggregates
    print("Aggregate Metrics:")
    print(evaluator.get_aggregate_metrics())
