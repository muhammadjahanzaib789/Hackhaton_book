# RAG Retrieval Evaluation Metrics: Implementation Patterns Research

## Table of Contents
1. [Evaluation Metrics Overview](#evaluation-metrics-overview)
2. [Metric Implementations](#metric-implementations)
3. [Test Harness Design](#test-harness-design)
4. [Python Libraries](#python-libraries)
5. [Benchmark Dataset Formats](#benchmark-dataset-formats)
6. [Best Practices](#best-practices)

---

## Evaluation Metrics Overview

RAG systems require both **order-agnostic metrics** (evaluate relevance count) and **order-aware metrics** (evaluate ranking quality). Here's the metric taxonomy:

### Order-Agnostic Metrics
- **Hit Rate@K**: Binary - did at least one relevant result appear in top-k?
- **Precision@K**: Percentage of top-k results that are relevant
  - Formula: `|relevant ∩ retrieved_k| / k`
  - Use when: evaluating early precision

- **Recall@K**: Percentage of all relevant documents in top-k
  - Formula: `|relevant ∩ retrieved_k| / |relevant|`
  - Use when: measuring completeness of retrieval

- **F1@K**: Harmonic mean of Precision@K and Recall@K
  - Formula: `2 * (Precision@K * Recall@K) / (Precision@K + Recall@K)`

### Order-Aware Metrics (Ranking Quality)
- **MRR@K** (Mean Reciprocal Rank): Position of first relevant result
  - Formula: `1 / rank_of_first_relevant` (0 if none in top-k)
  - Use when: first relevant result matters most (e.g., single-answer queries)

- **MAP@K** (Mean Average Precision): Precision at each relevant rank position, averaged
  - Assumes binary relevance
  - Use when: multiple relevant documents per query matter equally

- **NDCG@K** (Normalized Discounted Cumulative Gain): Graded relevance with position discount
  - Handles relevance grades (0, 1, 2, 3...)
  - Use when: relevance varies in degree (some docs better than others)
  - **RECOMMENDED for modern RAG systems** - used on MTEB Leaderboard

---

## Metric Implementations

### 1. Precision@K Implementation

```python
def precision_at_k(retrieved_ids, relevant_ids, k=5):
    """
    Calculate Precision@K.

    Args:
        retrieved_ids: List of document IDs in retrieval order
        relevant_ids: Set of relevant document IDs
        k: Cutoff position

    Returns:
        float: Precision@K (0.0-1.0)
    """
    top_k = set(retrieved_ids[:k])
    relevant_in_k = top_k.intersection(relevant_ids)
    return len(relevant_in_k) / k if k > 0 else 0.0


# Example:
retrieved = ['doc1', 'doc3', 'doc5', 'doc7', 'doc9']
relevant = {'doc1', 'doc5', 'doc10'}
precision_5 = precision_at_k(retrieved, relevant, k=5)
# Result: 2/5 = 0.4 (2 relevant docs in top-5)
```

### 2. Recall@K Implementation

```python
def recall_at_k(retrieved_ids, relevant_ids, k=5):
    """
    Calculate Recall@K.

    Args:
        retrieved_ids: List of document IDs in retrieval order
        relevant_ids: Set of relevant document IDs
        k: Cutoff position

    Returns:
        float: Recall@K (0.0-1.0)
    """
    top_k = set(retrieved_ids[:k])
    relevant_in_k = top_k.intersection(relevant_ids)
    return len(relevant_in_k) / len(relevant_ids) if len(relevant_ids) > 0 else 0.0


# Example:
retrieved = ['doc1', 'doc3', 'doc5', 'doc7', 'doc9']
relevant = {'doc1', 'doc5', 'doc10'}
recall_5 = recall_at_k(retrieved, relevant, k=5)
# Result: 2/3 = 0.667 (2 of 3 relevant docs found in top-5)
```

### 3. Mean Reciprocal Rank (MRR) Implementation

```python
def mrr_at_k(retrieved_ids, relevant_ids, k=10):
    """
    Calculate Mean Reciprocal Rank@K.

    MRR focuses on the rank position of the first relevant result.

    Args:
        retrieved_ids: List of document IDs in retrieval order
        relevant_ids: Set of relevant document IDs
        k: Cutoff position

    Returns:
        float: MRR@K (0.0-1.0)
    """
    for i, doc_id in enumerate(retrieved_ids[:k], start=1):
        if doc_id in relevant_ids:
            return 1.0 / i
    return 0.0  # No relevant document found


def mean_mrr(all_queries, k=10):
    """
    Calculate Mean MRR across multiple queries.

    Args:
        all_queries: List of (retrieved_ids, relevant_ids) tuples
        k: Cutoff position

    Returns:
        float: Average MRR across queries
    """
    mrr_scores = [mrr_at_k(retr, rel, k) for retr, rel in all_queries]
    return sum(mrr_scores) / len(mrr_scores) if mrr_scores else 0.0


# Example:
retrieved = ['doc2', 'doc4', 'doc1', 'doc7']
relevant = {'doc1', 'doc5'}
mrr_score = mrr_at_k(retrieved, relevant, k=10)
# Result: 1/3 = 0.333 (first relevant is at rank 3)
```

### 4. Mean Average Precision (MAP) Implementation

```python
def average_precision(retrieved_ids, relevant_ids, k=None):
    """
    Calculate Average Precision for a single query.

    AP = sum(precision@i for i where doc_i is relevant) / total_relevant

    Args:
        retrieved_ids: List of document IDs in retrieval order
        relevant_ids: Set of relevant document IDs
        k: Optional cutoff position

    Returns:
        float: Average Precision (0.0-1.0)
    """
    if not relevant_ids:
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


def mean_average_precision(all_queries, k=None):
    """
    Calculate Mean Average Precision across multiple queries.

    Args:
        all_queries: List of (retrieved_ids, relevant_ids) tuples
        k: Optional cutoff position

    Returns:
        float: MAP score
    """
    ap_scores = [average_precision(retr, rel, k) for retr, rel in all_queries]
    return sum(ap_scores) / len(ap_scores) if ap_scores else 0.0


# Example:
# Results: [Relevant, Non-relevant, Non-relevant, Relevant, Non-relevant]
retrieved = ['d1', 'd2', 'd3', 'd4', 'd5']
relevant = {'d1', 'd4', 'd6'}

# Precision@1 = 1/1 = 1.0 (d1 is relevant)
# Precision@4 = 2/4 = 0.5 (d4 is relevant)
# AP = (1.0 + 0.5) / 3 = 0.5
ap = average_precision(retrieved, relevant)
# Result: 0.5
```

### 5. NDCG (Normalized Discounted Cumulative Gain) Implementation

```python
import numpy as np

def ndcg_at_k(retrieved_relevances, k=10):
    """
    Calculate NDCG@K for a single query.

    NDCG = DCG@K / IDCG@K
    where:
      DCG@K = sum(rel_i / log2(i+1)) for i in 1..k
      IDCG@K = DCG of ideal ranking (sorted relevances)

    Args:
        retrieved_relevances: List of relevance scores (0-3 for graded, or binary 0/1)
                             in retrieval order
        k: Cutoff position

    Returns:
        float: NDCG@K (0.0-1.0)
    """
    # Truncate to top-k
    retrieved = retrieved_relevances[:k] if k else retrieved_relevances

    # Calculate DCG
    dcg = 0.0
    for i, relevance in enumerate(retrieved, start=1):
        discount = np.log2(i + 1)
        dcg += relevance / discount

    # Calculate Ideal DCG (perfect ranking)
    ideal = sorted(retrieved_relevances, reverse=True)[:k]
    idcg = 0.0
    for i, relevance in enumerate(ideal, start=1):
        discount = np.log2(i + 1)
        idcg += relevance / discount

    # Normalize
    if idcg == 0:
        return 0.0
    return dcg / idcg


def mean_ndcg(all_queries, k=10):
    """
    Calculate Mean NDCG across multiple queries.

    Args:
        all_queries: List of relevance score lists (each in retrieval order)
        k: Cutoff position

    Returns:
        float: Mean NDCG
    """
    ndcg_scores = [ndcg_at_k(relevances, k) for relevances in all_queries]
    return sum(ndcg_scores) / len(ndcg_scores) if ndcg_scores else 0.0


# Example with graded relevance (0-3):
# Graded: Perfect rank = [3, 2, 1, 0, 0...]
# Retrieved: [2, 0, 3, 1, 0...]
retrieved_relevances = [2, 0, 3, 1, 0]
ndcg_5 = ndcg_at_k(retrieved_relevances, k=5)
# DCG = 2/log2(2) + 0/log2(3) + 3/log2(4) + 1/log2(5)
#     ≈ 2.0 + 0 + 1.5 + 0.43 ≈ 3.93
# IDCG = 3/log2(2) + 2/log2(3) + 1/log2(4) + 0/log2(5)
#      ≈ 3.0 + 1.26 + 0.5 + 0 ≈ 4.76
# NDCG@5 ≈ 3.93 / 4.76 ≈ 0.826
```

---

## Test Harness Design

### 1. Benchmark Structure

A robust RAG evaluation harness requires:

```python
from dataclasses import dataclass
from typing import Dict, List, Set, Optional
import json

@dataclass
class Query:
    """Single benchmark query."""
    query_id: str
    text: str
    metadata: Optional[Dict] = None

@dataclass
class Document:
    """Single document in corpus."""
    doc_id: str
    text: str
    metadata: Optional[Dict] = None

@dataclass
class RelevanceJudgment:
    """Ground truth relevance judgment."""
    query_id: str
    doc_id: str
    relevance_grade: int  # 0 = not relevant, 1 = somewhat relevant, 2 = relevant, 3 = highly relevant
    judge: Optional[str] = None
    timestamp: Optional[str] = None

class BenchmarkDataset:
    """Container for benchmark data."""

    def __init__(self):
        self.queries: Dict[str, Query] = {}
        self.documents: Dict[str, Document] = {}
        self.qrels: Dict[str, Dict[str, int]] = {}  # query_id -> {doc_id -> relevance}

    def add_query(self, query: Query):
        """Add query to benchmark."""
        self.queries[query.query_id] = query

    def add_document(self, doc: Document):
        """Add document to corpus."""
        self.documents[doc.doc_id] = doc

    def add_judgment(self, judgment: RelevanceJudgment):
        """Add relevance judgment."""
        if judgment.query_id not in self.qrels:
            self.qrels[judgment.query_id] = {}
        self.qrels[judgment.query_id][judgment.doc_id] = judgment.relevance_grade

    def to_trec_qrels(self) -> str:
        """Export to TREC qrels format (query_id iter doc_id relevance)."""
        lines = []
        for query_id in sorted(self.qrels.keys()):
            for doc_id in sorted(self.qrels[query_id].keys()):
                rel = self.qrels[query_id][doc_id]
                lines.append(f"{query_id} 0 {doc_id} {rel}")
        return "\n".join(lines)

    def to_json(self, filepath: str):
        """Export dataset to JSON format."""
        data = {
            "queries": {qid: {"text": q.text, "metadata": q.metadata}
                       for qid, q in self.queries.items()},
            "documents": {did: {"text": d.text, "metadata": d.metadata}
                         for did, d in self.documents.items()},
            "qrels": self.qrels
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    @classmethod
    def from_json(cls, filepath: str) -> 'BenchmarkDataset':
        """Load dataset from JSON format."""
        with open(filepath) as f:
            data = json.load(f)

        dataset = cls()
        for qid, qdata in data.get("queries", {}).items():
            dataset.add_query(Query(qid, qdata["text"], qdata.get("metadata")))

        for did, ddata in data.get("documents", {}).items():
            dataset.add_document(Document(did, ddata["text"], ddata.get("metadata")))

        for qid, judgments in data.get("qrels", {}).items():
            for did, rel in judgments.items():
                dataset.add_judgment(RelevanceJudgment(qid, did, rel))

        return dataset

    @classmethod
    def from_trec_qrels(cls, qrels_file: str, queries_file: str = None, docs_file: str = None):
        """Load from TREC qrels format."""
        dataset = cls()

        # Parse qrels (format: query_id iter doc_id relevance)
        with open(qrels_file) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 4:
                    query_id, _, doc_id, relevance = parts[0], parts[1], parts[2], int(parts[3])
                    dataset.add_judgment(RelevanceJudgment(query_id, doc_id, relevance))

        return dataset


# Example usage:
dataset = BenchmarkDataset()

# Add queries
dataset.add_query(Query("q1", "What is machine learning?"))
dataset.add_query(Query("q2", "How do transformers work?"))

# Add documents
dataset.add_document(Document("d1", "Machine learning is a subset of AI..."))
dataset.add_document(Document("d2", "Transformers use attention mechanisms..."))

# Add judgments
dataset.add_judgment(RelevanceJudgment("q1", "d1", 2))  # d1 relevant to q1
dataset.add_judgment(RelevanceJudgment("q2", "d2", 3))  # d2 highly relevant to q2

# Export
print(dataset.to_trec_qrels())
dataset.to_json("benchmark.json")
```

### 2. Ground Truth Mapping Format

**TREC Qrels Format** (Standard):
```
query_id iteration doc_id relevance_grade
```

Example:
```
1 0 doc001 3
1 0 doc002 1
1 0 doc003 0
2 0 doc001 2
2 0 doc004 1
```

**JSON Format** (Recommended for RAG):
```json
{
  "queries": {
    "q1": {
      "text": "What is machine learning?",
      "metadata": {
        "source": "domain_specific",
        "category": "fundamentals"
      }
    }
  },
  "documents": {
    "d1": {
      "text": "Machine learning is...",
      "metadata": {
        "chapter": "Chapter 1",
        "page": 42
      }
    }
  },
  "qrels": {
    "q1": {
      "d1": 3,
      "d2": 1,
      "d3": 0
    }
  }
}
```

### 3. Evaluation Runner

```python
from typing import List, Tuple, Dict
import json
from datetime import datetime

class EvaluationResult:
    """Container for evaluation results."""

    def __init__(self, query_id: str, metrics: Dict[str, float]):
        self.query_id = query_id
        self.metrics = metrics
        self.timestamp = datetime.now().isoformat()

    def to_dict(self) -> Dict:
        return {
            "query_id": self.query_id,
            "metrics": self.metrics,
            "timestamp": self.timestamp
        }


class RetrievalEvaluator:
    """Main evaluation harness."""

    def __init__(self, dataset: BenchmarkDataset):
        self.dataset = dataset
        self.results = []

    def evaluate_run(self,
                    retrieved_results: Dict[str, List[str]],
                    metrics: List[str] = None,
                    k_values: List[int] = None) -> Dict[str, float]:
        """
        Evaluate a retrieval run against ground truth.

        Args:
            retrieved_results: {query_id -> [doc_id, doc_id, ...]} (in rank order)
            metrics: Which metrics to compute ('precision', 'recall', 'mrr', 'map', 'ndcg')
            k_values: Cutoff positions (e.g., [5, 10, 100])

        Returns:
            Aggregated metrics dict
        """
        if metrics is None:
            metrics = ['ndcg', 'mrr', 'map']
        if k_values is None:
            k_values = [5, 10]

        aggregated_metrics = {}

        for query_id in self.dataset.queries.keys():
            if query_id not in retrieved_results:
                continue

            retrieved_ids = retrieved_results[query_id]
            relevant_ids = set(self.dataset.qrels.get(query_id, {}).keys())

            # Get relevance scores for NDCG
            relevance_scores = []
            for doc_id in retrieved_ids:
                rel = self.dataset.qrels.get(query_id, {}).get(doc_id, 0)
                relevance_scores.append(rel)

            # Compute metrics at each k
            for k in k_values:
                if 'precision' in metrics:
                    prec = precision_at_k(retrieved_ids, relevant_ids, k)
                    aggregated_metrics[f"precision@{k}"] = aggregated_metrics.get(f"precision@{k}", 0) + prec

                if 'recall' in metrics:
                    rec = recall_at_k(retrieved_ids, relevant_ids, k)
                    aggregated_metrics[f"recall@{k}"] = aggregated_metrics.get(f"recall@{k}", 0) + rec

                if 'mrr' in metrics:
                    mrr = mrr_at_k(retrieved_ids, relevant_ids, k)
                    aggregated_metrics[f"mrr@{k}"] = aggregated_metrics.get(f"mrr@{k}", 0) + mrr

                if 'map' in metrics:
                    ap = average_precision(retrieved_ids, relevant_ids, k)
                    aggregated_metrics[f"map@{k}"] = aggregated_metrics.get(f"map@{k}", 0) + ap

                if 'ndcg' in metrics:
                    ndcg = ndcg_at_k(relevance_scores, k)
                    aggregated_metrics[f"ndcg@{k}"] = aggregated_metrics.get(f"ndcg@{k}", 0) + ndcg

        # Average across queries
        num_queries = len(self.dataset.queries)
        for key in aggregated_metrics:
            aggregated_metrics[key] /= num_queries

        return aggregated_metrics

    def save_results(self, filepath: str):
        """Save evaluation results to JSON."""
        with open(filepath, 'w') as f:
            json.dump([r.to_dict() for r in self.results], f, indent=2)


# Example usage:
evaluator = RetrievalEvaluator(dataset)

# Simulated retrieval results
retrieved_results = {
    "q1": ["d1", "d3", "d2"],  # ranked documents
    "q2": ["d2", "d1", "d3"]
}

metrics = evaluator.evaluate_run(
    retrieved_results,
    metrics=['precision', 'recall', 'mrr', 'map', 'ndcg'],
    k_values=[5, 10]
)

print(json.dumps(metrics, indent=2))
```

---

## Python Libraries

### 1. pytrec_eval (Recommended for TREC Compatibility)

**Installation:**
```bash
pip install pytrec_eval
```

**Key advantages:**
- ~10x faster than subprocess calls to trec_eval
- Standard TREC evaluation metrics
- Handles qrel and run formats directly

**Example:**
```python
import pytrec_eval
import json

# Ground truth (qrels format)
qrel = {
    'q1': {
        'd1': 1,
        'd2': 0,
        'd3': 1,
    },
    'q2': {
        'd1': 1,
        'd2': 1,
        'd3': 0,
    },
}

# Retrieval results (run format)
run = {
    'q1': {
        'd1': 1.5,  # scores can be floats
        'd2': 1.0,
        'd3': 0.5,
    },
    'q2': {
        'd1': 2.0,
        'd2': 1.2,
        'd3': 0.8,
    }
}

# Evaluate
evaluator = pytrec_eval.RelevanceEvaluator(
    qrel,
    {'map', 'ndcg', 'recall', 'precision'}
)

results = evaluator.evaluate(run)
print(json.dumps(results, indent=1))

# Output example:
# {
#  "q1": {
#   "map": 1.0,
#   "ndcg": 0.938,
#   "recall_10": 1.0,
#   "precision_5": 0.4
#  },
#  ...
# }
```

### 2. ranx (Modern, Blazing Fast)

**Installation:**
```bash
pip install ranx
```

**Key advantages:**
- Extremely fast (Numba-accelerated)
- Simple, modern API
- Multiple input formats (JSON, TREC, DataFrame)
- Built-in comparison and statistical testing
- Fusion algorithms support

**Example:**
```python
from ranx import Qrels, Run, evaluate, compare

# Define qrels
qrels = Qrels({
    "q1": {
        "d1": 3,
        "d2": 0,
        "d3": 2,
    }
})

# Define run
run = Run({
    "q1": {
        "d1": 1.5,
        "d2": 1.0,
        "d3": 0.5,
    }
})

# Evaluate
metrics = evaluate(
    qrels,
    run,
    metrics=["ndcg@5", "map@5", "mrr@5", "precision@5", "recall@5"]
)

print(metrics)
# Output: {'ndcg@5': 0.92, 'map@5': 0.95, ...}

# Compare multiple runs
metrics1 = evaluate(qrels, run1, metrics=["ndcg@10"])
metrics2 = evaluate(qrels, run2, metrics=["ndcg@10"])

comparison = compare(
    qrels,
    {"run1": run1, "run2": run2},
    metrics=["ndcg@10", "map@10"]
)
```

### 3. ir_measures (Unified Interface)

**Installation:**
```bash
pip install ir-measures
```

**Example:**
```python
from ir_measures import ndcg, map_measure, precision, recall

qrels = {...}  # qrels dict
run = {...}    # run dict

# Compute metrics
for query_measures in ndcg(qrels, run, [5, 10]):
    print(query_measures)  # Measure(query_id, ndcg@5)=0.92

# Compare queries
map_scores = list(map_measure(qrels, run, [10]))
```

### 4. rank_eval (Numba-Accelerated)

**Installation:**
```bash
pip install rank-eval```

```python
from rank_eval import evaluate

# Your ranked lists
ranked_lists = [[1, 2, 3], [3, 1, 2]]  # Relevance scores
ideal_ranked_lists = [[3, 2, 1], [3, 2, 1]]  # Ideal rankings

# Compute metrics
results = evaluate(
    ranked_lists,
    ideal_ranked_lists,
    metrics=['ndcg', 'mrr', 'map']
)
```

---

## Benchmark Dataset Formats

### Standard Benchmark Structure

**Directory Layout:**
```
benchmark_data/
├── queries.json          # All queries
├── documents.json        # Document corpus
├── qrels.json           # Ground truth relevance judgments
├── qrels.txt            # TREC format (alternative)
├── runs/                # Evaluation runs
│   ├── baseline.json
│   ├── model_v1.json
│   └── model_v2.json
├── results/             # Evaluation results
│   ├── baseline_scores.json
│   └── model_v1_scores.json
└── metadata.json        # Dataset metadata
```

### Metadata File (metadata.json)

```json
{
  "dataset_name": "Physical AI Book RAG",
  "version": "1.0",
  "created_date": "2025-01-10",
  "num_queries": 150,
  "num_documents": 1200,
  "num_judgments": 450,
  "relevance_scale": {
    "0": "not relevant",
    "1": "somewhat relevant",
    "2": "relevant",
    "3": "highly relevant"
  },
  "statistics": {
    "avg_judgments_per_query": 3.0,
    "max_judgments_per_query": 10,
    "binary_vs_graded": "graded",
    "source": "expert annotation"
  },
  "splits": {
    "train": {"num_queries": 100, "num_judgments": 300},
    "dev": {"num_queries": 25, "num_judgments": 75},
    "test": {"num_queries": 25, "num_judgments": 75}
  }
}
```

### Queries Format

```json
[
  {
    "id": "q1",
    "text": "What is machine learning?",
    "type": "factual",
    "difficulty": "easy",
    "domain": "ai_fundamentals"
  },
  {
    "id": "q2",
    "text": "Explain the transformer architecture and its advantages over RNNs",
    "type": "explanatory",
    "difficulty": "hard",
    "domain": "deep_learning"
  }
]
```

### Documents Format

```json
[
  {
    "id": "d1",
    "text": "Machine learning is a subset of artificial intelligence...",
    "source": "chapter_1.md",
    "page_number": 42,
    "section": "Introduction"
  },
  {
    "id": "d2",
    "text": "Transformers were introduced by Vaswani et al. in 2017...",
    "source": "chapter_3.md",
    "page_number": 120,
    "section": "Deep Learning Models"
  }
]
```

### Run Format

```json
{
  "q1": [
    {"id": "d1", "score": 0.95},
    {"id": "d3", "score": 0.87},
    {"id": "d5", "score": 0.72},
    {"id": "d2", "score": 0.65}
  ],
  "q2": [
    {"id": "d2", "score": 0.98},
    {"id": "d7", "score": 0.91},
    {"id": "d1", "score": 0.68}
  ]
}
```

Or simplified (ID only, preserves order):
```json
{
  "q1": ["d1", "d3", "d5", "d2"],
  "q2": ["d2", "d7", "d1"]
}
```

---

## Best Practices

### 1. Metric Selection Guidelines

| Use Case | Recommended Metric | Rationale |
|----------|-------------------|-----------|
| **First result matters** (QA, fact lookup) | MRR@K | Prioritizes early relevant results |
| **Comprehensive retrieval** (Multi-doc RAG) | Recall@K, MAP@K | Measures coverage of relevant docs |
| **Production RAG systems** | NDCG@K | Handles graded relevance, MTEB standard |
| **Ranking quality** | NDCG@K + Precision@5 | Combines ranking and early precision |
| **Balanced evaluation** | Precision@K + Recall@K + NDCG@K | Multi-dimensional view |

### 2. Reproducible Evaluation

```python
import random
import numpy as np
from datetime import datetime

class ReproducibleEvaluator:
    """Evaluation harness with full reproducibility."""

    def __init__(self, seed=42):
        self.seed = seed
        self.set_seed()
        self.eval_metadata = {
            "timestamp": datetime.now().isoformat(),
            "seed": seed,
            "numpy_version": np.__version__
        }

    def set_seed(self):
        """Set all random seeds."""
        random.seed(self.seed)
        np.random.seed(self.seed)

    def evaluate_with_metadata(self,
                               dataset: BenchmarkDataset,
                               retrieved_results: Dict[str, List[str]],
                               model_name: str = None,
                               model_version: str = None):
        """
        Evaluate and capture full provenance.
        """
        results = {
            "metadata": {
                **self.eval_metadata,
                "model_name": model_name,
                "model_version": model_version,
                "dataset_version": getattr(dataset, 'version', 'unknown'),
                "dataset_size": {
                    "queries": len(dataset.queries),
                    "documents": len(dataset.documents),
                    "judgments": sum(len(j) for j in dataset.qrels.values())
                }
            },
            "metrics": self.evaluate_run(dataset, retrieved_results),
            "query_results": {}
        }

        # Per-query breakdown
        for qid in dataset.queries:
            if qid in retrieved_results:
                results["query_results"][qid] = {
                    "query": dataset.queries[qid].text,
                    "retrieved": retrieved_results[qid],
                    "relevant": list(dataset.qrels.get(qid, {}).keys())
                }

        return results
```

### 3. Handle Edge Cases

```python
def safe_metric_computation(retrieved_ids, relevant_ids, k=5):
    """
    Safely compute metrics with edge case handling.
    """
    # Empty retrieval
    if not retrieved_ids:
        return {
            "precision@k": 0.0,
            "recall@k": 0.0,
            "mrr@k": 0.0,
            "issue": "Empty retrieval results"
        }

    # No relevant documents
    if not relevant_ids:
        return {
            "precision@k": 0.0,
            "recall@k": 0.0,  # Undefined, set to 0
            "mrr@k": 0.0,
            "issue": "No relevant documents found"
        }

    # Compute normally
    return {
        "precision@k": precision_at_k(retrieved_ids, relevant_ids, k),
        "recall@k": recall_at_k(retrieved_ids, relevant_ids, k),
        "mrr@k": mrr_at_k(retrieved_ids, relevant_ids, k)
    }
```

### 4. Benchmark Best Practices

1. **Diverse Query Types**: Mix factual, explanatory, and comparison queries
2. **Relevant Document Range**: 1-10 relevant docs per query (realistic)
3. **Document Pool**: 50-200 candidates per query for fair evaluation
4. **Multiple Judges**: When possible, use 2-3 judges per query
5. **Graded Relevance**: Use 4-scale (0-3) for nuanced evaluation
6. **Train/Dev/Test Split**: 60-20-20 or 70-15-15 split
7. **Version Control**: Track dataset versions and judge agreements

### 5. Production Monitoring

```python
class MetricsMonitor:
    """Monitor retrieval metrics over time."""

    def __init__(self, alert_threshold=0.05):
        self.metrics_history = []
        self.alert_threshold = alert_threshold

    def log_metrics(self, metrics: Dict[str, float], timestamp: str = None):
        """Log metrics for trend analysis."""
        if timestamp is None:
            timestamp = datetime.now().isoformat()

        self.metrics_history.append({
            "timestamp": timestamp,
            "metrics": metrics
        })

    def detect_regression(self, metric_name: str = "ndcg@10"):
        """Detect metric regression over recent runs."""
        if len(self.metrics_history) < 2:
            return None

        current = self.metrics_history[-1]["metrics"].get(metric_name)
        previous = self.metrics_history[-2]["metrics"].get(metric_name)

        if current and previous:
            regression = (previous - current) / previous
            if regression > self.alert_threshold:
                return {
                    "metric": metric_name,
                    "regression_pct": regression * 100,
                    "alert": True
                }

        return None
```

---

## Implementation Checklist

- [ ] Choose metrics based on use case (NDCG recommended for RAG)
- [ ] Define ground truth format (TREC qrels or JSON)
- [ ] Create benchmark dataset with diverse queries
- [ ] Implement evaluation harness with proper aggregation
- [ ] Test edge cases (empty results, no relevant docs)
- [ ] Set up reproducibility (seeds, metadata capture)
- [ ] Document metric formulas and assumptions
- [ ] Compare with established benchmarks (BEIR, MS MARCO)
- [ ] Monitor metrics in production
- [ ] Version control dataset and evaluation code

---

## References

- [NDCG Explanation](https://www.evidentlyai.com/ranking-metrics/ndcg-metric)
- [Mean Average Precision (MAP)](https://vitalflux.com/mean-average-precision-map-for-information-retrieval-systems/)
- [Retrieval Evaluation Metrics Overview](https://weaviate.io/blog/retrieval-evaluation-metrics)
- [RAG Evaluation Metrics 2025](https://futureagi.com/blogs/rag-evaluation-metrics-2025)
- [TREC qrels Format](https://faculty.washington.edu/levow/courses/ling573_SPR2011/hw/trec_eval_desc.htm)
- [pytrec_eval GitHub](https://github.com/cvangysel/pytrec_eval)
- [ranx Library](https://github.com/AmenRa/ranx)
- [BEIR Benchmark](https://github.com/beir-cellar/beir)
- [MS MARCO Dataset](https://microsoft.github.io/msmarco/)
- [TREC Deep Learning Track](https://trec-rag.github.io/)
