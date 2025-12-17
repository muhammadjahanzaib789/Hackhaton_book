# RAG Evaluation Metrics - Practical Usage Examples

## Quick Start

### 1. Basic Metric Computation

```python
from src.evaluation.metrics import (
    precision_at_k, recall_at_k, mrr_at_k,
    average_precision_at_k, ndcg_at_k
)

# Your retrieval results and ground truth
retrieved_docs = ['doc1', 'doc3', 'doc5', 'doc7', 'doc9']
relevant_docs = {'doc1', 'doc5', 'doc10'}

# Compute individual metrics
precision_5 = precision_at_k(retrieved_docs, relevant_docs, k=5)
# Output: 0.4 (2 relevant out of 5)

recall_5 = recall_at_k(retrieved_docs, relevant_docs, k=5)
# Output: 0.667 (2 out of 3 relevant docs found)

mrr_score = mrr_at_k(retrieved_docs, relevant_docs, k=10)
# Output: 0.5 (first relevant doc at rank 2)

ap_score = average_precision_at_k(retrieved_docs, relevant_docs, k=5)
# Output: 0.65

# For NDCG, use relevance scores (graded: 0-3)
relevance_scores = [1, 0, 1, 0, 0]  # Relevance of each retrieved doc
ndcg_5 = ndcg_at_k(relevance_scores, k=5)
# Output: 0.826
```

### 2. Batch Evaluation with Multiple Queries

```python
from src.evaluation.metrics import BatchEvaluator

# Create evaluator
evaluator = BatchEvaluator(k_values=[5, 10])

# Evaluate multiple queries
evaluator.evaluate_query(
    query_id="q1",
    retrieved_ids=['d1', 'd3', 'd5', 'd7'],
    relevant_ids={'d1', 'd5', 'd10'},
    relevance_scores=[2, 0, 3, 0],
    metrics=['precision', 'recall', 'mrr', 'map', 'ndcg']
)

evaluator.evaluate_query(
    query_id="q2",
    retrieved_ids=['d2', 'd4', 'd1'],
    relevant_ids={'d1', 'd2'},
    relevance_scores=[1, 0, 2],
    metrics=['precision', 'recall', 'mrr', 'map', 'ndcg']
)

# Get aggregate metrics
aggregate_metrics = evaluator.get_aggregate_metrics()
print("Aggregate Metrics:")
print(aggregate_metrics)
# Output:
# {
#   'precision@5': 0.45,
#   'recall@5': 0.65,
#   'mrr@5': 0.65,
#   'map@5': 0.75,
#   'ndcg@5': 0.82,
#   'precision@10': 0.45,
#   ...
# }
```

### 3. Create and Manage Benchmark Dataset

```python
from src.evaluation.benchmark import (
    BenchmarkDataset, Query, Document, RelevanceJudgment
)

# Create benchmark
benchmark = BenchmarkDataset("Physical AI Book RAG", version="1.0")

# Add queries
benchmark.add_query(Query(
    query_id="q1",
    text="What is machine learning?",
    query_type="factual",
    difficulty="easy",
    domain="ai_fundamentals"
))

benchmark.add_query(Query(
    query_id="q2",
    text="Explain the transformer architecture",
    query_type="explanatory",
    difficulty="hard",
    domain="deep_learning"
))

# Add documents
benchmark.add_document(Document(
    doc_id="d1",
    text="Machine learning is a subset of AI...",
    source="chapter_1.md",
    page_number=42,
    section="Introduction"
))

benchmark.add_document(Document(
    doc_id="d2",
    text="Transformers use attention mechanisms...",
    source="chapter_3.md",
    page_number=120,
    section="Deep Learning Models"
))

# Add relevance judgments (graded: 0-3)
benchmark.add_judgment(RelevanceJudgment("q1", "d1", 3))  # Highly relevant
benchmark.add_judgment(RelevanceJudgment("q1", "d2", 0))  # Not relevant
benchmark.add_judgment(RelevanceJudgment("q2", "d2", 3))  # Highly relevant

# Get statistics
stats = benchmark.get_statistics()
print(stats)
# Output:
# {
#   'num_queries': 2,
#   'num_documents': 2,
#   'total_judgments': 3,
#   'avg_judgments_per_query': 1.5,
#   ...
# }

# Save to JSON
benchmark.save_json("benchmark.json")

# Export to TREC format
trec_qrels = benchmark.to_trec_qrels()
print(trec_qrels)
# Output:
# q1 0 d1 3
# q1 0 d2 0
# q2 0 d2 3

benchmark.save_trec_qrels("qrels.txt")
```

### 4. Load Existing Benchmark

```python
# From JSON
benchmark = BenchmarkDataset.from_json("benchmark.json")

# From TREC qrels
benchmark = BenchmarkDataset.from_trec_qrels("qrels.txt", name="Loaded Benchmark")

# Query operations
relevant_to_q1 = benchmark.get_relevant_docs("q1")
# Output: {'d1', ...}

relevance_dict = benchmark.get_relevant_docs_graded("q1")
# Output: {'d1': 3, 'd2': 0, ...}

grade = benchmark.get_relevance_grade("q1", "d1")
# Output: 3
```

### 5. Full Evaluation Pipeline (Reproducible)

```python
from src.evaluation.benchmark import ReproducibleEvaluationRunner
from src.evaluation import metrics

# Load benchmark
benchmark = BenchmarkDataset.from_json("benchmark.json")

# Create evaluation runner
runner = ReproducibleEvaluationRunner(
    dataset=benchmark,
    model_name="rag-embedder-v1",
    model_version="1.2.3"
)

# Simulated retrieval results from your RAG system
retrieval_results = {
    "q1": ["d1", "d3", "d5"],      # Retrieved docs in rank order
    "q2": ["d2", "d4", "d1"],
    # ... more queries
}

# Run evaluation
results = runner.evaluate_run(retrieval_results, metrics)

# Results include:
# {
#   "metadata": {
#     "timestamp": "2025-01-10T...",
#     "model": "rag-embedder-v1",
#     "model_version": "1.2.3",
#     "dataset": "Physical AI Book RAG",
#     "dataset_version": "1.0",
#     "dataset_size": {...}
#   },
#   "metrics": {
#     "ndcg@5": 0.82,
#     "map@5": 0.75,
#     ...
#   },
#   "per_query_results": [
#     {
#       "query_id": "q1",
#       "retrieved_doc_ids": [...],
#       "relevant_doc_ids": [...],
#       "metrics": {...}
#     },
#     ...
#   ]
# }

# Save results for analysis
runner.save_results("evaluation_results.json")
```

### 6. Compare Multiple Runs

```python
from src.evaluation.benchmark import BenchmarkComparison
import json

# Load multiple evaluation results
with open("run1_results.json") as f:
    run1 = json.load(f)

with open("run2_results.json") as f:
    run2 = json.load(f)

runs = {
    "baseline": run1,
    "model_v1": run2
}

# Compare
comparison = BenchmarkComparison.compare_runs(runs)
print(comparison)
# Output:
# {
#   "runs": ["baseline", "model_v1"],
#   "metric_comparison": {
#     "ndcg@5": {"baseline": 0.75, "model_v1": 0.82},
#     "map@5": {"baseline": 0.70, "model_v1": 0.78},
#     ...
#   }
# }

# Find best run
best_run, best_score = BenchmarkComparison.find_best_run(runs, metric="ndcg@10")
print(f"Best run: {best_run} with NDCG@10={best_score:.3f}")
```

### 7. Advanced: Per-Query Analysis

```python
import json

# Load results
with open("evaluation_results.json") as f:
    results = json.load(f)

# Analyze per-query performance
for query_result in results["per_query_results"]:
    query_id = query_result["query_id"]
    metrics = query_result["metrics"]

    ndcg = metrics.get("ndcg@10", 0)
    mrr = metrics.get("mrr@10", 0)

    if ndcg < 0.5:  # Low-performing queries
        print(f"Low-performing query {query_id}: NDCG@10={ndcg:.3f}")
        print(f"  Retrieved: {query_result['retrieved_doc_ids']}")
        print(f"  Relevant: {query_result['relevant_doc_ids']}")
```

### 8. Detect Regression in Production

```python
from src.evaluation.metrics import detect_metric_anomalies

# Previous evaluation metrics
previous = {"ndcg@10": 0.85, "map@10": 0.80}

# Current evaluation metrics
current = {"ndcg@10": 0.78, "map@10": 0.75}

# Detect anomalies (threshold = 5% change)
anomalies = detect_metric_anomalies(current, previous, threshold=0.05)
print(anomalies)
# Output:
# {
#   'ndcg@10': -0.082 (8.2% degradation),
#   'map@10': -0.0625 (6.25% degradation)
# }

if anomalies:
    print("ALERT: Metrics have regressed!")
```

## Complete Integration Example

```python
"""
Complete workflow: Create benchmark, evaluate runs, compare results.
"""

from src.evaluation.benchmark import (
    BenchmarkDataset, Query, Document, RelevanceJudgment,
    ReproducibleEvaluationRunner, BenchmarkComparison
)
from src.evaluation import metrics
import json

# Step 1: Create benchmark dataset
print("Creating benchmark...")
benchmark = BenchmarkDataset("Physical AI RAG", version="1.0")

# Add queries (would normally load from external source)
queries_data = [
    ("q1", "What is machine learning?", "factual", "easy"),
    ("q2", "Explain transformers", "explanatory", "hard"),
]

for qid, text, qtype, difficulty in queries_data:
    benchmark.add_query(Query(qid, text, query_type=qtype, difficulty=difficulty))

# Add documents (would normally come from your corpus)
docs_data = [
    ("d1", "Machine learning is a subset of AI..."),
    ("d2", "Transformers use attention mechanisms..."),
    ("d3", "Neural networks are fundamental..."),
]

for did, text in docs_data:
    benchmark.add_document(Document(did, text))

# Add judgments (would normally come from annotators)
benchmark.add_judgment(RelevanceJudgment("q1", "d1", 3))
benchmark.add_judgment(RelevanceJudgment("q1", "d3", 1))
benchmark.add_judgment(RelevanceJudgment("q2", "d2", 3))
benchmark.add_judgment(RelevanceJudgment("q2", "d3", 1))

benchmark.save_json("benchmark.json")
print(f"Benchmark created: {benchmark.get_statistics()}")

# Step 2: Evaluate multiple runs
print("\nEvaluating runs...")

runs = {
    "baseline": {
        "q1": ["d1", "d3", "d2"],
        "q2": ["d2", "d1", "d3"],
    },
    "model_v1": {
        "q1": ["d1", "d2", "d3"],
        "q2": ["d2", "d3", "d1"],
    }
}

results = {}
for run_name, retrieval_results in runs.items():
    runner = ReproducibleEvaluationRunner(
        benchmark, model_name=run_name, model_version="1.0"
    )
    result = runner.evaluate_run(retrieval_results, metrics)
    results[run_name] = result
    runner.save_results(f"results_{run_name}.json")
    print(f"  {run_name}: NDCG@10={result['metrics'].get('ndcg@10', 0):.3f}")

# Step 3: Compare results
print("\nComparing runs...")
comparison = BenchmarkComparison.compare_runs(results)

print("\nMetric Comparison:")
for metric, scores in comparison["metric_comparison"].items():
    print(f"  {metric}:")
    for run_name, score in scores.items():
        if score is not None:
            print(f"    {run_name}: {score:.3f}")

# Find best
best_run, best_score = BenchmarkComparison.find_best_run(results, "ndcg@10")
print(f"\nBest run: {best_run} (NDCG@10={best_score:.3f})")
```

## Dataset Format Examples

### JSON Benchmark Format
```json
{
  "metadata": {
    "dataset_name": "Physical AI RAG",
    "version": "1.0",
    "created_date": "2025-01-10T..."
  },
  "statistics": {
    "num_queries": 150,
    "num_documents": 1200,
    "total_judgments": 450
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
    "q1": {
      "d1": 3,
      "d2": 0
    }
  }
}
```

### TREC Qrels Format
```
q1 0 d1 3
q1 0 d2 0
q2 0 d2 3
q2 0 d3 1
```

### Evaluation Results Format
```json
{
  "metadata": {
    "timestamp": "2025-01-10T...",
    "model": "rag-embedder-v1",
    "model_version": "1.2.3",
    "dataset": "Physical AI Book RAG",
    "dataset_version": "1.0"
  },
  "metrics": {
    "ndcg@5": 0.82,
    "ndcg@10": 0.85,
    "map@5": 0.75,
    "mrr@10": 0.80
  },
  "per_query_results": [
    {
      "query_id": "q1",
      "retrieved_doc_ids": ["d1", "d3", "d5"],
      "relevant_doc_ids": ["d1", "d5", "d10"],
      "metrics": {
        "ndcg@10": 0.92,
        "mrr@10": 0.5
      }
    }
  ]
}
```

## Testing

```python
# Run unit tests
import unittest
from src.evaluation import metrics

class TestMetrics(unittest.TestCase):
    def test_precision_at_k(self):
        retrieved = ['d1', 'd3', 'd5']
        relevant = {'d1', 'd5', 'd10'}
        self.assertAlmostEqual(
            metrics.precision_at_k(retrieved, relevant, k=3),
            2/3
        )

    def test_ndcg_perfect_ranking(self):
        # Perfect ranking should give NDCG=1.0
        scores = [3, 2, 1]
        self.assertAlmostEqual(
            metrics.ndcg_at_k(scores, k=3),
            1.0
        )

if __name__ == '__main__':
    unittest.main()
```
