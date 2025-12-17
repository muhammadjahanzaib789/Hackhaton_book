# Data Model: Integrated RAG Chatbot

**Feature**: 001-integrated-rag-chatbot
**Date**: 2025-12-17
**Phase**: 1 - Design & Contracts

## Overview

This document defines all data entities, their fields, relationships, validation rules, and state transitions for the RAG chatbot system.

---

## Entity Catalog

### 1. BookContentChunk
Represents a segment of the book (paragraph, section, or page) with associated text, embedding vector, and metadata.

**Storage**: Qdrant (vector) + Neon Postgres (metadata)

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `chunk_id` | UUID | Primary key, required | Unique identifier for the chunk |
| `text` | Text | Required, 100-2000 chars | The actual text content of the chunk |
| `embedding_vector` | Float[] | Required, 768 dimensions | Vector embedding from Qwen3 Embedding 8B |
| `chapter_name` | String | Required, max 200 chars | Chapter title (e.g., "Chapter 1: Introduction to ROS 2") |
| `section_name` | String | Optional, max 200 chars | Section or subsection title |
| `page_number` | Integer | Optional, >= 1 | Page number in the book (if applicable) |
| `document_path` | String | Required, max 500 chars | Path to the source Markdown file (e.g., "docs/module-01/lesson-01.md") |
| `chunk_index` | Integer | Required, >= 0 | Sequential index of chunk within the document |
| `token_count` | Integer | Required, 100-500 | Number of tokens in the chunk |
| `created_at` | Timestamp | Required, auto-set | When the chunk was indexed |
| `metadata` | JSON | Optional | Additional metadata (tags, topics, code_block: boolean, etc.) |

**Validation Rules**:
- `text` length must be between 100 and 2000 characters (enforces chunking quality)
- `token_count` must match actual token count of `text` (±5% tolerance)
- `embedding_vector` must have exactly 768 dimensions (Qwen3 Embedding 8B output)
- `document_path` must exist in the `physical-ai-book/docs/` directory

**Relationships**:
- One `BookContentChunk` corresponds to zero or many `SourceCitation` (via `chunk_id`)

**State Transitions**:
- **Created** → chunk indexed and stored in Qdrant + Postgres
- **Updated** → if book content changes, chunk is re-indexed (new `created_at`, same `chunk_id`)
- **Deleted** → chunk removed from both Qdrant and Postgres (cascades to `SourceCitation`)

---

### 2. ChatSession
Represents a user's interaction session with the chatbot.

**Storage**: Neon Postgres

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `session_id` | UUID | Primary key, required | Unique identifier for the session |
| `user_id` | String | Optional, max 100 chars | Anonymous user identifier (e.g., browser fingerprint, IP hash) |
| `started_at` | Timestamp | Required, auto-set | When the session started |
| `last_activity_at` | Timestamp | Required, auto-update | Last query timestamp in this session |
| `mode` | Enum | Required, default "full_book" | "full_book" or "selected_text" |
| `query_count` | Integer | Required, >= 0, default 0 | Number of queries in this session |
| `status` | Enum | Required, default "active" | "active", "ended", "abandoned" |

**Validation Rules**:
- `mode` must be one of: "full_book", "selected_text"
- `status` must be one of: "active", "ended", "abandoned"
- `last_activity_at` must be >= `started_at`
- `query_count` must match the number of `Query` records linked to this session

**Relationships**:
- One `ChatSession` has many `Query` (via `session_id`)

**State Transitions**:
- **active** → Session is ongoing, accepting new queries
- **ended** → User explicitly closed the chat widget
- **abandoned** → No activity for 30+ minutes (automatic transition)

---

### 3. Query
Represents a user's question submitted to the chatbot.

**Storage**: Neon Postgres

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `query_id` | UUID | Primary key, required | Unique identifier for the query |
| `session_id` | UUID | Foreign key, required | References `ChatSession.session_id` |
| `query_text` | Text | Required, 5-500 chars | The user's question |
| `selected_text` | Text | Optional, max 5000 chars | Text selected by the user (if mode="selected_text") |
| `mode` | Enum | Required | "full_book" or "selected_text" |
| `timestamp` | Timestamp | Required, auto-set | When the query was submitted |
| `processing_time_ms` | Integer | Optional, >= 0 | Total processing time in milliseconds |
| `error` | String | Optional, max 500 chars | Error message if query failed |

**Validation Rules**:
- `query_text` length must be between 5 and 500 characters
- If `mode` is "selected_text", `selected_text` must not be null and must be >= 10 characters
- If `mode` is "full_book", `selected_text` must be null
- `processing_time_ms` should be < 5000ms under normal conditions (warning threshold)

**Relationships**:
- One `Query` belongs to one `ChatSession` (via `session_id`)
- One `Query` has exactly one `Response` (via `query_id`)

**State Transitions**:
- **Submitted** → Query received by backend
- **Processing** → Embedding generated, retrieval in progress
- **Completed** → Response generated successfully
- **Failed** → Error occurred (`error` field populated)

---

### 4. Response
Represents the chatbot's answer to a query.

**Storage**: Neon Postgres

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `response_id` | UUID | Primary key, required | Unique identifier for the response |
| `query_id` | UUID | Foreign key, required, unique | References `Query.query_id` (1-to-1) |
| `response_text` | Text | Required, 50-2000 chars | The generated answer |
| `confidence_score` | Float | Optional, 0.0-1.0 | Confidence score from LLM (if available) |
| `model_used` | String | Required, max 100 chars | LLM model name (e.g., "gpt-4", "claude-sonnet-3.5") |
| `timestamp` | Timestamp | Required, auto-set | When the response was generated |
| `token_count` | Integer | Optional, >= 0 | Number of tokens in the response |

**Validation Rules**:
- `response_text` length must be between 50 and 2000 characters
- `confidence_score` must be between 0.0 and 1.0 (if provided)
- `model_used` must match one of the supported OpenRouter models

**Relationships**:
- One `Response` belongs to one `Query` (via `query_id`)
- One `Response` has many `SourceCitation` (via `response_id`)

**State Transitions**:
- **Generated** → Response created by LLM
- **Delivered** → Response sent to the client
- **Rated** → User provided feedback (future enhancement)

---

### 5. SourceCitation
Represents a reference to book content used to generate a response.

**Storage**: Neon Postgres

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `citation_id` | UUID | Primary key, required | Unique identifier for the citation |
| `response_id` | UUID | Foreign key, required | References `Response.response_id` |
| `chunk_id` | UUID | Foreign key, required | References `BookContentChunk.chunk_id` |
| `chapter_name` | String | Required, max 200 chars | Chapter title (denormalized for quick access) |
| `section_name` | String | Optional, max 200 chars | Section title (denormalized) |
| `page_number` | Integer | Optional, >= 1 | Page number (denormalized) |
| `relevance_score` | Float | Required, 0.0-1.0 | Similarity score from vector search |
| `excerpt` | Text | Optional, max 500 chars | Short snippet of the cited text |
| `link` | String | Optional, max 500 chars | URL anchor to the exact location in the book (e.g., "/docs/module-01/lesson-01#section-2") |

**Validation Rules**:
- `relevance_score` must be between 0.0 and 1.0
- Citations should be ordered by `relevance_score` DESC when displayed to the user
- At least 1 citation must exist per `Response` (enforced at application level)

**Relationships**:
- One `SourceCitation` belongs to one `Response` (via `response_id`)
- One `SourceCitation` references one `BookContentChunk` (via `chunk_id`)

**State Transitions**:
- **Created** → Citation generated during retrieval phase
- **Displayed** → Citation shown to the user in the chat interface

---

## Entity Relationship Diagram (ERD)

```text
┌─────────────────────┐
│   ChatSession       │
│ ─────────────────── │
│ session_id (PK)     │
│ user_id             │
│ started_at          │
│ last_activity_at    │
│ mode                │
│ query_count         │
│ status              │
└──────────┬──────────┘
           │
           │ 1:N
           │
           ▼
┌─────────────────────┐
│      Query          │
│ ─────────────────── │
│ query_id (PK)       │
│ session_id (FK)     │
│ query_text          │
│ selected_text       │
│ mode                │
│ timestamp           │
│ processing_time_ms  │
│ error               │
└──────────┬──────────┘
           │
           │ 1:1
           │
           ▼
┌─────────────────────┐
│     Response        │
│ ─────────────────── │
│ response_id (PK)    │
│ query_id (FK)       │
│ response_text       │
│ confidence_score    │
│ model_used          │
│ timestamp           │
│ token_count         │
└──────────┬──────────┘
           │
           │ 1:N
           │
           ▼
┌─────────────────────┐         ┌─────────────────────┐
│  SourceCitation     │         │ BookContentChunk    │
│ ─────────────────── │         │ ─────────────────── │
│ citation_id (PK)    │   N:1   │ chunk_id (PK)       │
│ response_id (FK)    │────────▶│ text                │
│ chunk_id (FK)       │         │ embedding_vector    │
│ chapter_name        │         │ chapter_name        │
│ section_name        │         │ section_name        │
│ page_number         │         │ page_number         │
│ relevance_score     │         │ document_path       │
│ excerpt             │         │ chunk_index         │
│ link                │         │ token_count         │
└─────────────────────┘         │ created_at          │
                                │ metadata            │
                                └─────────────────────┘
```

---

## Storage Strategy

### Qdrant (Vector Database)
**What**: Vector embeddings for semantic search
**Entities**: `BookContentChunk.embedding_vector`
**Collection**: "book_chunks"
**Configuration**:
- Vector size: 768 (Qwen3 Embedding 8B)
- Distance metric: Cosine similarity
- HNSW indexing: M=16, ef_construct=100 (default)

### Neon Postgres (Relational Database)
**What**: Metadata, chat logs, source citations
**Entities**: `BookContentChunk` (metadata only), `ChatSession`, `Query`, `Response`, `SourceCitation`
**Schema**: "public"
**Indexes**:
- `BookContentChunk`: Index on `chapter_name`, `document_path`
- `ChatSession`: Index on `started_at`, `status`
- `Query`: Index on `session_id`, `timestamp`
- `Response`: Index on `query_id` (unique)
- `SourceCitation`: Index on `response_id`, `chunk_id`

---

## Scaling Considerations

**Current Scale (Free Tier)**:
- Qdrant: ~1,000 chunks (500-page book)
- Neon Postgres: ~10,000 queries/month, ~50,000 chat logs

**Growth Path**:
- If chunk count exceeds 1M vectors → Upgrade to Qdrant paid tier ($95/month for 10GB)
- If Postgres exceeds 0.5 GB → Upgrade to Neon Launch tier ($19/month for 10GB)
- If query volume exceeds 1,000 requests/day → Increase OpenRouter credits

---

## Migration Strategy

**Initial Schema Deployment**:
1. Create Neon Postgres database with schema (see `backend/src/db/migrations/001_initial_schema.sql`)
2. Create Qdrant collection "book_chunks" with 768-dimension vectors
3. Run content indexer to populate both databases

**Schema Evolution**:
- Use Alembic for Postgres migrations
- Qdrant schema changes require collection recreation (rare)
- Preserve backward compatibility for chat logs (historical data)

---

## Data Validation & Integrity

**Application-Level Checks**:
- Validate chunk token count matches text content
- Ensure at least 1 source citation per response
- Verify selected text length >= 10 chars when mode="selected_text"
- Check embedding vector dimensions = 768

**Database-Level Constraints**:
- Foreign key constraints (session_id, query_id, response_id, chunk_id)
- NOT NULL constraints on required fields
- CHECK constraints on enums (mode, status)
- Unique constraint on `Response.query_id` (1-to-1 relationship)

**Failure Modes**:
- **Qdrant down**: Return error "Vector search unavailable, try again later"
- **Neon down**: Return error "Chat history unavailable, query processing disabled"
- **Embedding generation fails**: Return error "Unable to process query, check input format"
- **LLM timeout**: Return error "Response generation took too long, please try again"

---

## Next Steps (Phase 1 Continued)

1. Generate API contracts based on this data model
2. Define request/response schemas for `/api/query`, `/api/index` endpoints
3. Create quickstart.md with local development setup instructions
