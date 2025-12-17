# RAG Retrieval Evaluation Module

Complete implementation of information retrieval evaluation metrics and test harness for RAG (Retrieval-Augmented Generation) systems.

## Overview

This module provides production-ready implementations of standard IR metrics aligned with TREC and MTEB standards. It's designed for evaluating retrieval quality in RAG pipelines at scale.

### Key Features

- **Order-Agnostic Metrics**: Precision@K, Recall@K, F1@K, Hit Rate@K
- **Order-Aware Metrics**: MRR (Mean Reciprocal Rank), MAP (Mean Average Precision)
- **Graded Metrics**: NDCG@K (Normalized Discounted Cumulative Gain) - **recommended for RAG**
- **Benchmark Management**: Dataset creation, persistence, multi-format support
- **Reproducible Evaluation**: Full provenance tracking with metadata
- **Production Monitoring**: Regression detection and comparison tools

## Architecture

```
evaluation/
├── __init__.py              # Public API
├── metrics.py              # All metric implementations
├── benchmark.py            # Dataset and evaluation management
├── EXAMPLE_USAGE.md         # Practical examples
└── README.md               # This file
```

### Module Structure

**metrics.py** (~600 lines):
- Single metric functions: `precision_at_k()`, `recall_at_k()`, `ndcg_at_k()`, etc.
- Batch aggregation: `mean_precision_at_k()`, `mean_ndcg_at_k()`, etc.
- `BatchEvaluator` class: Multi-query, multi-metric evaluation engine
- Utility functions: anomaly detection, metric conversion

**benchmark.py** (~450 lines):
- Data models: `Query`, `Document`, `RelevanceJudgment`
- `BenchmarkDataset`: Complete dataset management
  - JSON format support (import/export)
  - TREC qrels format support (import/export)
  - Statistics and analysis
- `ReproducibleEvaluationRunner`: Full evaluation pipeline with version tracking
- `BenchmarkComparison`: Compare multiple runs

## Quick Start

### Installation

Place this module in your backend:
```
backend/src/evaluation/
```

### Basic Usage

```python
from src.evaluation.metrics import ndcg_at_k, BatchEvaluator
from src.evaluation.benchmark import BenchmarkDataset, Query, Document

# Single metric
relevance_scores = [3, 0, 2, 1, 0]  # Graded relevance 0-3
ndcg = ndcg_at_k(relevance_scores, k=5)
print(f"NDCG@5: {ndcg:.3f}")

# Batch evaluation
evaluator = BatchEvaluator(k_values=[5, 10])
evaluator.evaluate_query(
    query_id="q1",
    retrieved_ids=["d1", "d3", "d5"],
    relevant_ids={"d1", "d5", "d10"},
    relevance_scores=[3, 0, 2],
)
print(evaluator.get_aggregate_metrics())

# Full benchmark workflow
benchmark = BenchmarkDataset("MyBenchmark", "1.0")
benchmark.add_query(Query("q1", "What is ML?"))
benchmark.add_document(Document("d1", "ML is..."))
benchmark.save_json("benchmark.json")
```

See **EXAMPLE_USAGE.md** for complete examples.

## Metrics Reference

### When to Use Each Metric

| Metric | Use Case | Notes |
|--------|----------|-------|
| **Precision@K** | Early precision matters | What % of top-K are relevant? |
| **Recall@K** | Comprehensive retrieval | What % of all relevant docs in top-K? |
| **MRR@K** | Single-answer queries | Position of first relevant result |
| **MAP@K** | Multi-doc retrieval, binary relevance | Average precision across queries |
| **NDCG@K** | **Recommended for RAG** | Graded relevance, ranking quality |

### Formulas

**Precision@K**:
```
Precision@K = |relevant ∩ retrieved_k| / k
```

**Recall@K**:
```
Recall@K = |relevant ∩ retrieved_k| / |relevant|
```

**MRR@K**:
```
MRR@K = 1 / rank_of_first_relevant (or 0 if none)
```

**MAP@K**:
```
AP = sum(Precision@i where doc_i is relevant) / |relevant|
MAP@K = average AP across all queries
```

**NDCG@K** (Recommended):
```
DCG@K = sum(rel_i / log2(i+1)) for i in 1..k
IDCG@K = DCG of ideal ranking
NDCG@K = DCG@K / IDCG@K
```

## Dataset Formats

### JSON Format (Recommended)

```json
{
  "metadata": {
    "dataset_name": "Physical AI RAG",
    "version": "1.0",
    "created_date": "2025-01-10"
  },
  "queries": {
    "q1": {
      "query_id": "q1",
      "text": "What is machine learning?",
      "query_type": "factual",
      "difficulty": "easy"
    }
  },
  "documents": {
    "d1": {
      "doc_id": "d1",
      "text": "Machine learning is...",
      "source": "chapter_1.md",
      "page_number": 42
    }
  },
  "qrels": {
    "q1": {"d1": 3, "d2": 0}
  }
}
```

### TREC Qrels Format

```
query_id iter doc_id relevance_grade
q1 0 d1 3
q1 0 d2 0
q2 0 d2 3
```

## Evaluation Pipeline

```python
# 1. Create or load benchmark
benchmark = BenchmarkDataset.from_json("benchmark.json")

# 2. Run retrieval (your RAG system)
retrieval_results = {
    "q1": ["d1", "d3", "d5"],  # In rank order
    "q2": ["d2", "d4", "d1"],
}

# 3. Evaluate
runner = ReproducibleEvaluationRunner(
    dataset=benchmark,
    model_name="rag-v1",
    model_version="1.0"
)
results = runner.evaluate_run(retrieval_results, metrics)
runner.save_results("results.json")

# 4. Compare multiple runs
comparison = BenchmarkComparison.compare_runs({
    "baseline": baseline_results,
    "model_v1": results
})
```

## Production Usage

### Monitor Metrics Over Time

```python
from src.evaluation.metrics import detect_metric_anomalies

# Previous
previous_metrics = {"ndcg@10": 0.85, "map@10": 0.80}

# Current
current_metrics = {"ndcg@10": 0.78, "map@10": 0.75}

# Detect regression
anomalies = detect_metric_anomalies(
    current_metrics,
    previous_metrics,
    threshold=0.05  # 5% change threshold
)

if anomalies:
    logger.warning(f"Metric regression detected: {anomalies}")
    # Alert on-call team, rollback, etc.
```

### Batch Evaluation

```python
evaluator = BatchEvaluator(k_values=[5, 10, 100])

# Evaluate 1000s of queries
for query_id, retrieved_ids in retrieval_results.items():
    relevant_ids = benchmark.get_relevant_docs(query_id)
    relevance_scores = [
        benchmark.get_relevance_grade(query_id, doc_id)
        for doc_id in retrieved_ids
    ]

    evaluator.evaluate_query(
        query_id=query_id,
        retrieved_ids=retrieved_ids,
        relevant_ids=relevant_ids,
        relevance_scores=relevance_scores
    )

# Get aggregate metrics
metrics = evaluator.get_aggregate_metrics()
```

## Supported Libraries

This module is designed to be compatible with:
- **pytrec_eval**: TREC evaluation standard (10x faster than CLI)
- **ranx**: Modern Numba-accelerated library
- **ir_measures**: Unified IR metrics interface
- **rank_eval**: High-performance ranking metrics

See main research document for comparison and integration guides.

## Integration with Existing Code

### With retrieval.py

```python
from src.evaluation.metrics import BatchEvaluator
from src.services.retrieval import retrieval_service

# Evaluate retrieval performance
evaluator = BatchEvaluator(k_values=[5, 10])

for query_id, relevant_ids in benchmark.qrels.items():
    # Get retrieval results
    results = await retrieval_service.retrieve(
        query=benchmark.queries[query_id].text,
        top_k=10
    )

    retrieved_ids = [r.chunk_id for r in results]
    relevance_scores = [
        benchmark.get_relevance_grade(query_id, str(chunk_id))
        for chunk_id in retrieved_ids
    ]

    evaluator.evaluate_query(
        query_id=query_id,
        retrieved_ids=[str(cid) for cid in retrieved_ids],
        relevant_ids=relevant_ids,
        relevance_scores=relevance_scores
    )

metrics = evaluator.get_aggregate_metrics()
```

## Data Models

### Query
```python
@dataclass
class Query:
    query_id: str
    text: str
    query_type: Optional[str] = None      # "factual", "explanatory", etc.
    difficulty: Optional[str] = None      # "easy", "medium", "hard"
    domain: Optional[str] = None          # "ai_fundamentals", etc.
    metadata: Optional[Dict] = None
```

### Document
```python
@dataclass
class Document:
    doc_id: str
    text: str
    source: Optional[str] = None          # Source file
    page_number: Optional[int] = None
    section: Optional[str] = None
    metadata: Optional[Dict] = None
```

### RelevanceJudgment
```python
@dataclass
class RelevanceJudgment:
    query_id: str
    doc_id: str
    relevance_grade: int                  # 0-3 scale
    judge: Optional[str] = None
    timestamp: Optional[str] = None
```

## Performance Considerations

### Batch Evaluation

For large datasets (1000s of queries):

```python
# Efficient: Pre-compute and cache
benchmark = BenchmarkDataset.from_json("benchmark.json")
relevant_cache = {
    q: benchmark.get_relevant_docs(q)
    for q in benchmark.queries
}

evaluator = BatchEvaluator(k_values=[5, 10])
for query_id, retrieved_ids in results.items():
    evaluator.evaluate_query(
        query_id=query_id,
        retrieved_ids=retrieved_ids,
        relevant_ids=relevant_cache[query_id],
        # ... relevance scores
    )
```

### Memory Usage

- Single metric computation: O(k) where k is cutoff
- Batch evaluation: O(n * k) where n is number of queries
- NDCG computation uses numpy for efficiency

## Error Handling

All metrics handle edge cases:

```python
# Empty retrieval
precision_at_k([], {"d1"}, k=5)  # Returns 0.0

# No relevant documents
recall_at_k(["d1"], set(), k=5)  # Returns 0.0

# No results at all
ndcg_at_k([], k=10)  # Returns 0.0
```

## Testing

```python
import unittest
from src.evaluation import metrics

class TestMetrics(unittest.TestCase):
    def test_ndcg_perfect_ranking(self):
        """Perfect ranking should give NDCG=1.0"""
        scores = [3, 2, 1]
        self.assertAlmostEqual(metrics.ndcg_at_k(scores, k=3), 1.0)

    def test_precision_basic(self):
        retrieved = ["d1", "d3", "d5"]
        relevant = {"d1", "d5", "d10"}
        self.assertAlmostEqual(
            metrics.precision_at_k(retrieved, relevant, k=3),
            2/3
        )

if __name__ == "__main__":
    unittest.main()
```

## Best Practices

1. **Use NDCG@K for RAG**: Standard on MTEB Leaderboard, handles graded relevance
2. **Combine metrics**: Precision@5 (early quality) + Recall@10 (coverage)
3. **Create diverse queries**: Mix factual, explanatory, comparison types
4. **Use graded relevance**: 0-3 scale is more informative than binary
5. **Version everything**: Dataset version, model version, evaluation timestamp
6. **Monitor regressions**: Track metrics over time in production
7. **Export results**: Save TREC format for reproducibility

## References

- [NDCG Explanation](https://www.evidentlyai.com/ranking-metrics/ndcg-metric)
- [Mean Average Precision](https://vitalflux.com/mean-average-precision-map-for-information-retrieval-systems/)
- [Retrieval Evaluation Overview](https://weaviate.io/blog/retrieval-evaluation-metrics)
- [TREC Evaluation Standards](https://trec.nist.gov/)
- [BEIR Benchmark](https://github.com/beir-cellar/beir)
- [pytrec_eval](https://github.com/cvangysel/pytrec_eval)
- [ranx Library](https://github.com/AmenRa/ranx)

## API Reference

### metrics.py

**Single Query Metrics:**
- `precision_at_k(retrieved_ids, relevant_ids, k)`
- `recall_at_k(retrieved_ids, relevant_ids, k)`
- `f1_at_k(retrieved_ids, relevant_ids, k)`
- `hit_rate_at_k(retrieved_ids, relevant_ids, k)`
- `mrr_at_k(retrieved_ids, relevant_ids, k)`
- `average_precision_at_k(retrieved_ids, relevant_ids, k)`
- `ndcg_at_k(relevance_scores, k)`

**Batch Aggregation:**
- `mean_precision_at_k(all_queries, k)`
- `mean_recall_at_k(all_queries, k)`
- `mean_mrr_at_k(all_queries, k)`
- `mean_average_precision_at_k(all_queries, k)`
- `mean_ndcg_at_k(all_queries, k)`

**Classes:**
- `BatchEvaluator(k_values)` - Multi-query evaluation engine

### benchmark.py

**Dataset Management:**
- `BenchmarkDataset(name, version)` - Dataset container
- `BenchmarkDataset.from_json(filepath)` - Load from JSON
- `BenchmarkDataset.from_trec_qrels(filepath)` - Load from TREC
- `benchmark.save_json(filepath)` - Export to JSON
- `benchmark.save_trec_qrels(filepath)` - Export to TREC
- `benchmark.get_statistics()` - Dataset stats

**Evaluation:**
- `ReproducibleEvaluationRunner(dataset, model_name)` - Full pipeline
- `runner.evaluate_run(retrieval_results, metrics_module)` - Run evaluation
- `runner.save_results(filepath)` - Save results

**Comparison:**
- `BenchmarkComparison.compare_runs(runs_results)` - Compare multiple runs
- `BenchmarkComparison.find_best_run(runs_results, metric)` - Find best

## License

Part of the Loop RAG project.
