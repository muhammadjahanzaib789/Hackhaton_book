"""
Benchmark Test Harness for RAG Evaluation

Provides structured benchmark dataset management, ground truth handling,
and reproducible evaluation workflow for RAG retrieval systems.

Features:
- Benchmark dataset creation and persistence (JSON/TREC formats)
- Qrels (query relevance judgments) management
- Reproducible evaluation with version tracking
- Multi-metric aggregation and comparison
"""

import json
import logging
from dataclasses import dataclass, asdict
from typing import Dict, List, Set, Optional, Tuple
from datetime import datetime
from pathlib import Path

logger = logging.getLogger(__name__)


# ============================================================================
# Data Models for Benchmark Components
# ============================================================================


@dataclass
class Query:
    """Represents a single query in the benchmark."""
    query_id: str
    text: str
    query_type: Optional[str] = None  # e.g., "factual", "explanatory"
    difficulty: Optional[str] = None  # e.g., "easy", "medium", "hard"
    domain: Optional[str] = None      # e.g., "ai_fundamentals"
    metadata: Optional[Dict] = None

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class Document:
    """Represents a document in the corpus."""
    doc_id: str
    text: str
    source: Optional[str] = None       # e.g., "chapter_1.md"
    page_number: Optional[int] = None
    section: Optional[str] = None      # e.g., "Introduction"
    metadata: Optional[Dict] = None

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class RelevanceJudgment:
    """Ground truth relevance judgment for query-document pair."""
    query_id: str
    doc_id: str
    relevance_grade: int  # 0=not relevant, 1=somewhat, 2=relevant, 3=highly relevant
    judge: Optional[str] = None
    timestamp: Optional[str] = None

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class BenchmarkMetadata:
    """Metadata about benchmark dataset."""
    dataset_name: str
    version: str
    created_date: str
    num_queries: int
    num_documents: int
    num_judgments: int
    relevance_scale: Dict[int, str] = None
    statistics: Optional[Dict] = None
    splits: Optional[Dict] = None

    def __post_init__(self):
        if self.relevance_scale is None:
            self.relevance_scale = {
                0: "not relevant",
                1: "somewhat relevant",
                2: "relevant",
                3: "highly relevant"
            }

    def to_dict(self) -> Dict:
        return asdict(self)


# ============================================================================
# Benchmark Dataset Management
# ============================================================================


class BenchmarkDataset:
    """
    Container for benchmark data with support for multiple formats.

    Manages:
    - Queries and documents
    - Ground truth relevance judgments (qrels)
    - Metadata and statistics
    - Import/export in various formats
    """

    def __init__(self, name: str = "Default Benchmark", version: str = "1.0"):
        """Initialize benchmark dataset."""
        self.name = name
        self.version = version
        self.queries: Dict[str, Query] = {}
        self.documents: Dict[str, Document] = {}
        self.qrels: Dict[str, Dict[str, int]] = {}  # query_id -> {doc_id -> relevance}
        self.created_date = datetime.now().isoformat()

    def add_query(self, query: Query) -> None:
        """Add query to benchmark."""
        if query.query_id in self.queries:
            logger.warning(f"Query {query.query_id} already exists, overwriting")
        self.queries[query.query_id] = query

    def add_document(self, doc: Document) -> None:
        """Add document to corpus."""
        if doc.doc_id in self.documents:
            logger.warning(f"Document {doc.doc_id} already exists, overwriting")
        self.documents[doc.doc_id] = doc

    def add_judgment(self, judgment: RelevanceJudgment) -> None:
        """Add relevance judgment."""
        if judgment.query_id not in self.qrels:
            self.qrels[judgment.query_id] = {}
        self.qrels[judgment.query_id][judgment.doc_id] = judgment.relevance_grade

    def add_judgments_bulk(self, judgments: List[RelevanceJudgment]) -> None:
        """Add multiple judgments efficiently."""
        for judgment in judgments:
            self.add_judgment(judgment)

    def get_relevant_docs(self, query_id: str) -> Set[str]:
        """Get set of relevant document IDs for a query."""
        return set(self.qrels.get(query_id, {}).keys())

    def get_relevant_docs_graded(self, query_id: str) -> Dict[str, int]:
        """Get relevant documents with their relevance grades."""
        return self.qrels.get(query_id, {})

    def get_relevance_grade(self, query_id: str, doc_id: str) -> int:
        """Get relevance grade for query-doc pair (0 if not found)."""
        return self.qrels.get(query_id, {}).get(doc_id, 0)

    def get_statistics(self) -> Dict:
        """Compute dataset statistics."""
        total_judgments = sum(len(docs) for docs in self.qrels.values())

        judgments_per_query = []
        for query_id in self.queries:
            num_judgments = len(self.qrels.get(query_id, {}))
            judgments_per_query.append(num_judgments)

        return {
            "num_queries": len(self.queries),
            "num_documents": len(self.documents),
            "total_judgments": total_judgments,
            "avg_judgments_per_query": (
                sum(judgments_per_query) / len(judgments_per_query)
                if judgments_per_query else 0
            ),
            "max_judgments_per_query": max(judgments_per_query) if judgments_per_query else 0,
            "min_judgments_per_query": min(judgments_per_query) if judgments_per_query else 0,
        }

    # ========================================================================
    # TREC Format Support
    # ========================================================================

    def to_trec_qrels(self) -> str:
        """
        Export to TREC qrels format.

        Format: query_id iter doc_id relevance_grade
        (iter is typically 0, used for compatibility)
        """
        lines = []
        for query_id in sorted(self.qrels.keys()):
            for doc_id in sorted(self.qrels[query_id].keys()):
                rel = self.qrels[query_id][doc_id]
                lines.append(f"{query_id} 0 {doc_id} {rel}")
        return "\n".join(lines)

    def save_trec_qrels(self, filepath: str) -> None:
        """Save qrels in TREC format."""
        with open(filepath, 'w') as f:
            f.write(self.to_trec_qrels())
        logger.info(f"Saved qrels to {filepath}")

    @classmethod
    def from_trec_qrels(cls, qrels_file: str, name: str = "TREC Benchmark") -> 'BenchmarkDataset':
        """
        Load benchmark from TREC qrels file.

        Args:
            qrels_file: Path to qrels file
            name: Dataset name

        Returns:
            BenchmarkDataset instance
        """
        dataset = cls(name=name)

        with open(qrels_file) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 4:
                    query_id = parts[0]
                    doc_id = parts[2]
                    relevance = int(parts[3])
                    dataset.add_judgment(RelevanceJudgment(query_id, doc_id, relevance))

        logger.info(f"Loaded {len(dataset.qrels)} queries from {qrels_file}")
        return dataset

    # ========================================================================
    # JSON Format Support
    # ========================================================================

    def to_json(self, include_docs: bool = True) -> Dict:
        """
        Convert to JSON-compatible dictionary.

        Args:
            include_docs: Include full document texts (can be large)

        Returns:
            Dictionary with queries, documents, and qrels
        """
        data = {
            "metadata": {
                "dataset_name": self.name,
                "version": self.version,
                "created_date": self.created_date,
            },
            "statistics": self.get_statistics(),
            "qrels": self.qrels
        }

        if include_docs:
            data["queries"] = {
                qid: q.to_dict() for qid, q in self.queries.items()
            }
            data["documents"] = {
                did: d.to_dict() for did, d in self.documents.items()
            }
        else:
            # Only query texts and IDs
            data["queries"] = {
                qid: {"query_id": qid, "text": q.text}
                for qid, q in self.queries.items()
            }
            data["document_ids"] = list(self.documents.keys())

        return data

    def save_json(self, filepath: str, include_docs: bool = True) -> None:
        """
        Save benchmark to JSON file.

        Args:
            filepath: Output file path
            include_docs: Include full document texts
        """
        data = self.to_json(include_docs=include_docs)
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        logger.info(f"Saved benchmark to {filepath}")

    @classmethod
    def from_json(cls, filepath: str) -> 'BenchmarkDataset':
        """
        Load benchmark from JSON file.

        Args:
            filepath: Input file path

        Returns:
            BenchmarkDataset instance
        """
        with open(filepath) as f:
            data = json.load(f)

        # Create dataset
        meta = data.get("metadata", {})
        dataset = cls(
            name=meta.get("dataset_name", "Unknown"),
            version=meta.get("version", "1.0")
        )

        # Load queries
        for qid, qdata in data.get("queries", {}).items():
            dataset.add_query(Query(
                query_id=qdata.get("query_id", qid),
                text=qdata.get("text", ""),
                query_type=qdata.get("query_type"),
                difficulty=qdata.get("difficulty"),
                domain=qdata.get("domain"),
                metadata=qdata.get("metadata")
            ))

        # Load documents
        for did, ddata in data.get("documents", {}).items():
            dataset.add_document(Document(
                doc_id=ddata.get("doc_id", did),
                text=ddata.get("text", ""),
                source=ddata.get("source"),
                page_number=ddata.get("page_number"),
                section=ddata.get("section"),
                metadata=ddata.get("metadata")
            ))

        # Load qrels
        for qid, judgments in data.get("qrels", {}).items():
            for did, rel in judgments.items():
                dataset.add_judgment(RelevanceJudgment(qid, did, rel))

        logger.info(f"Loaded {len(dataset.qrels)} queries from {filepath}")
        return dataset

    # ========================================================================
    # Run Format Support (for evaluation results)
    # ========================================================================

    @staticmethod
    def run_to_json(runs: Dict[str, Dict[str, float]]) -> str:
        """
        Convert retrieval run to JSON format.

        Run format: {query_id -> {doc_id -> score}}
        """
        return json.dumps(runs, indent=2)

    @staticmethod
    def run_to_trec(runs: Dict[str, List[str]], run_name: str = "default") -> str:
        """
        Convert retrieval run to TREC format.

        TREC run format: query_id Q0 doc_id rank score run_name

        Args:
            runs: {query_id -> [doc_id, ...]} (in rank order)
            run_name: Name of the run

        Returns:
            String in TREC run format
        """
        lines = []
        for query_id in sorted(runs.keys()):
            for rank, doc_id in enumerate(runs[query_id], start=1):
                score = 1.0 / rank  # Default scoring: 1/rank
                lines.append(f"{query_id} Q0 {doc_id} {rank} {score:.6f} {run_name}")
        return "\n".join(lines)


# ============================================================================
# Evaluation Runner with Reproducibility
# ============================================================================


class ReproducibleEvaluationRunner:
    """
    Evaluation runner with full reproducibility and version tracking.

    Captures:
    - Dataset version
    - Model version
    - Metrics computed
    - Timestamp and environment info
    - Per-query results
    """

    def __init__(self, dataset: BenchmarkDataset, model_name: str = "Unknown", model_version: str = "1.0"):
        """Initialize runner."""
        self.dataset = dataset
        self.model_name = model_name
        self.model_version = model_version
        self.run_timestamp = datetime.now().isoformat()
        self.results = {}

    def evaluate_run(self, runs: Dict[str, List[str]], metrics_module) -> Dict:
        """
        Evaluate a retrieval run.

        Args:
            runs: {query_id -> [doc_id, ...]} in rank order
            metrics_module: Module with metric functions (e.g., metrics.py)

        Returns:
            Complete evaluation result with metadata and metrics
        """
        # Create evaluator
        evaluator = metrics_module.BatchEvaluator(k_values=[5, 10])

        # Evaluate each query
        for query_id in self.dataset.queries:
            if query_id not in runs:
                logger.warning(f"No results for query {query_id}")
                continue

            retrieved_ids = runs[query_id]
            relevant_ids = self.dataset.get_relevant_docs(query_id)
            relevance_scores = [
                self.dataset.get_relevance_grade(query_id, doc_id)
                for doc_id in retrieved_ids
            ]

            evaluator.evaluate_query(
                query_id=query_id,
                retrieved_ids=retrieved_ids,
                relevant_ids=relevant_ids,
                relevance_scores=relevance_scores,
                metrics=['precision', 'recall', 'mrr', 'map', 'ndcg']
            )

        # Aggregate metrics
        aggregate_metrics = evaluator.get_aggregate_metrics()

        # Build complete result
        result = {
            "metadata": {
                "timestamp": self.run_timestamp,
                "model": self.model_name,
                "model_version": self.model_version,
                "dataset": self.dataset.name,
                "dataset_version": self.dataset.version,
                "dataset_size": self.dataset.get_statistics()
            },
            "metrics": aggregate_metrics,
            "per_query_results": evaluator.get_results_json()
        }

        self.results = result
        return result

    def save_results(self, filepath: str) -> None:
        """Save evaluation results to JSON."""
        with open(filepath, 'w') as f:
            json.dump(self.results, f, indent=2)
        logger.info(f"Saved results to {filepath}")


# ============================================================================
# Benchmark Comparison and Analysis
# ============================================================================


class BenchmarkComparison:
    """Compare multiple evaluation runs."""

    @staticmethod
    def compare_runs(runs_results: Dict[str, Dict]) -> Dict:
        """
        Compare metrics across multiple runs.

        Args:
            runs_results: {run_name -> result_dict}

        Returns:
            Comparison summary
        """
        comparison = {
            "runs": list(runs_results.keys()),
            "metric_comparison": {}
        }

        # Extract all metrics
        all_metrics = set()
        for result in runs_results.values():
            if "metrics" in result:
                all_metrics.update(result["metrics"].keys())

        # Build comparison table
        for metric in sorted(all_metrics):
            comparison["metric_comparison"][metric] = {}
            for run_name, result in runs_results.items():
                value = result.get("metrics", {}).get(metric, None)
                comparison["metric_comparison"][metric][run_name] = value

        return comparison

    @staticmethod
    def find_best_run(runs_results: Dict[str, Dict], metric: str = "ndcg@10") -> Tuple[str, float]:
        """
        Find best performing run for a metric.

        Args:
            runs_results: {run_name -> result_dict}
            metric: Metric to compare on

        Returns:
            (best_run_name, best_score)
        """
        best_run = None
        best_score = -1

        for run_name, result in runs_results.items():
            score = result.get("metrics", {}).get(metric, -1)
            if score > best_score:
                best_score = score
                best_run = run_name

        return best_run, best_score


if __name__ == "__main__":
    # Example usage
    logging.basicConfig(level=logging.INFO)

    # Create benchmark
    benchmark = BenchmarkDataset("Physical AI RAG", "1.0")

    # Add queries
    benchmark.add_query(Query("q1", "What is machine learning?", domain="fundamentals"))
    benchmark.add_query(Query("q2", "Explain transformers", domain="deep_learning"))

    # Add documents
    benchmark.add_document(Document("d1", "ML is a subset of AI...", section="Intro"))
    benchmark.add_document(Document("d2", "Transformers use attention...", section="Models"))

    # Add judgments
    benchmark.add_judgment(RelevanceJudgment("q1", "d1", 2))
    benchmark.add_judgment(RelevanceJudgment("q2", "d2", 3))

    # Save and load
    benchmark.save_json("/tmp/benchmark.json")
    loaded = BenchmarkDataset.from_json("/tmp/benchmark.json")

    print("Statistics:", loaded.get_statistics())
