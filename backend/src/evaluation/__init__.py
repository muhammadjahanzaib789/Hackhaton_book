"""
RAG Evaluation Module

Provides complete infrastructure for evaluating RAG retrieval systems:
- Metric implementations (Precision@K, Recall@K, MRR, MAP, NDCG)
- Benchmark dataset management
- Test harness for reproducible evaluation
- Comparison and analysis tools

Main Classes:
- metrics.BatchEvaluator: Multi-metric evaluation engine
- benchmark.BenchmarkDataset: Benchmark data management
- benchmark.ReproducibleEvaluationRunner: Full evaluation pipeline
"""

from .metrics import (
    # Order-agnostic metrics
    precision_at_k,
    recall_at_k,
    f1_at_k,
    hit_rate_at_k,
    # Order-aware metrics
    mrr_at_k,
    average_precision_at_k,
    # Graded metrics
    dcg_at_k,
    ndcg_at_k,
    # Batch aggregation
    mean_precision_at_k,
    mean_recall_at_k,
    mean_mrr_at_k,
    mean_average_precision_at_k,
    mean_ndcg_at_k,
    # Engine
    BatchEvaluator,
    QueryEvaluationResult,
    MetricScore,
)

from .benchmark import (
    # Data models
    Query,
    Document,
    RelevanceJudgment,
    BenchmarkMetadata,
    # Dataset management
    BenchmarkDataset,
    # Evaluation runners
    ReproducibleEvaluationRunner,
    # Comparison
    BenchmarkComparison,
)

__all__ = [
    # Metrics - order-agnostic
    "precision_at_k",
    "recall_at_k",
    "f1_at_k",
    "hit_rate_at_k",
    # Metrics - order-aware
    "mrr_at_k",
    "average_precision_at_k",
    # Metrics - graded
    "dcg_at_k",
    "ndcg_at_k",
    # Batch aggregation
    "mean_precision_at_k",
    "mean_recall_at_k",
    "mean_mrr_at_k",
    "mean_average_precision_at_k",
    "mean_ndcg_at_k",
    # Engine classes
    "BatchEvaluator",
    "QueryEvaluationResult",
    "MetricScore",
    # Benchmark data models
    "Query",
    "Document",
    "RelevanceJudgment",
    "BenchmarkMetadata",
    # Benchmark management
    "BenchmarkDataset",
    "ReproducibleEvaluationRunner",
    "BenchmarkComparison",
]

__version__ = "1.0.0"
__author__ = "RAG Evaluation Team"
