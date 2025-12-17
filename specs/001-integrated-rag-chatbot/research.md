# Research Findings: Integrated RAG Chatbot

**Feature**: 001-integrated-rag-chatbot
**Date**: 2025-12-17
**Research Completed**: Phase 0

## Overview

This document resolves all NEEDS CLARIFICATION items from the Technical Context section of the implementation plan.

---

## 1. OpenRouter API Rate Limits & Qwen Embeddings

### Decision
Use **Qwen3 Embedding 8B** via DeepInfra or Nebius provider for embeddings, and purchase $10 in OpenRouter credits to unlock higher rate limits.

### Specifics
**Rate Limits:**
- Free tier (no credits): 50 requests/day, 20 RPM
- With $10+ credits: 1,000 requests/day, 20 RPM (permanent, even if balance drops)
- Paid models: No platform-level rate limits

**Qwen3 Embedding Models:**
- Qwen3 Embedding 8B: $0.00000001/M tokens (via DeepInfra/Nebius)
- Context length: 32,768 tokens
- OpenAI-compatible embeddings API
- #1 on MTEB leaderboard (June 2025)

**Pricing Structure:**
- 5.5% fee on credit purchases ($0.80 minimum)
- No markup on inference (pass-through pricing)

### Rationale
- Qwen3 Embedding 8B offers the best price-to-performance ratio for semantic search
- $10 credit purchase unlocks 1,000 requests/day, sufficient for initial deployment (10k queries/month estimate)
- OpenAI-compatible API simplifies integration

### Alternatives Considered
- Sentence-Transformers (local): Requires hosting costs, slower inference
- OpenAI text-embedding-3: More expensive, no significant quality advantage for technical content

---

## 2. Qdrant Cloud Free Tier Limits

### Decision
Use Qdrant Cloud Free Tier with 768-dimension embeddings.

### Specifics
**Free Tier Limits:**
- Storage: 1 GB total
- Capacity: ~1 million vectors at 768 dimensions
- Configuration: Single node
- Cost: Always free, no credit card required

**Capacity Planning for Our Use Case:**
- Technical book: ~500-1000 chunks (400 tokens each)
- Storage required: ~1-2 MB (with metadata)
- Free tier supports 200+ books worth of content

### Rationale
- Free tier is more than sufficient for a single textbook RAG implementation
- No collection number limits mentioned, allowing organization by chapter/topic
- Managed service eliminates vector database hosting complexity

### Alternatives Considered
- Self-hosted Qdrant: Requires server costs, maintenance overhead
- Pinecone: Free tier limited to 1 index, 100k vectors
- Weaviate Cloud: Free tier limited to 1GB, but less mature Python SDK

---

## 3. Neon Serverless Postgres Free Tier Limits

### Decision
Use Neon Free Tier for conversation history and document metadata only. Store actual document content/chunks in Qdrant.

### Specifics
**Free Tier Limits:**
- Storage: 0.5 GB per project (up to 20 projects)
- Compute: 100 CU-hours/month per project
- Egress: 5 GB/month
- Auto-scales to zero after 5 minutes inactivity (cold start ~500ms)

**Compute Hour Allocation:**
- At 0.25 CU: 400 hours/month (~13 hours/day uptime)
- At 0.5 CU: 200 hours/month (~6.5 hours/day)
- At 1.0 CU: 100 hours/month (~3.3 hours/day)

**For Our RAG Application:**
- Store: conversation logs, user sessions, chunk metadata (chapter, page, section)
- Expected usage: <100 MB for thousands of conversations
- Scale to 0.25 CU to maximize uptime

### Rationale
- 0.5 GB storage is sufficient for chat logs and metadata
- Hybrid approach (Postgres for metadata, Qdrant for vectors) keeps DB small and efficient
- Auto-scaling to zero reduces costs during low usage periods
- Cold start (~500ms) is acceptable for chat application

### Alternatives Considered
- Supabase: Free tier includes 500 MB, but Postgres-specific features not needed
- MongoDB Atlas: Free tier (512 MB), but relational data model preferred for chat logs
- SQLite (local): No remote access, complicates deployment

---

## 4. RAG Chunking Best Practices for Technical Books

### Decision
Use **recursive chunking** with 400-token chunks and 80-token overlap (20%). Preserve code blocks intact.

### Specifics
**Chunking Parameters:**
```python
CHUNK_SIZE = 400  # tokens
OVERLAP = 80      # 20% overlap
MIN_CHUNK_SIZE = 100  # Don't create tiny chunks
```

**Strategy-Specific Rules:**
1. **Code blocks**: Keep entire code examples together (don't split)
2. **Explanatory text**: 400-500 tokens with 20% overlap
3. **Diagrams/figures**: Store references with surrounding context (±200 tokens)
4. **Mathematical formulas**: Keep with explanation in same chunk

**Chunking Strategy Ranking (2025 Research):**
1. Semantic chunking (highest accuracy)
2. Recursive chunking (80% of RAG apps, balances simplicity with structure)
3. Page-level chunking (NVIDIA 2024 benchmark winner)

### Rationale
- **400 tokens**: Captures complete concept (intro + explanation + example)
- **20% overlap**: Ensures context preservation across chunk boundaries
- **Recursive approach**: Respects Markdown structure (headers, code blocks, paragraphs)
- Fits within Qwen3 Embedding's 32K context window with room to spare
- Allows 10-15 chunks to be retrieved and fit in 8K LLM context window

### Alternatives Considered
- Fixed-size chunking (256 tokens): May split concepts mid-explanation
- Sentence-based chunking: Too granular for technical content with code examples
- Page-level chunking: Too large (500+ tokens), reduces retrieval precision

---

## 5. FastAPI Deployment Options

### Decision
**Development**: AWS Lambda Free Tier
**Production**: Railway ($5/month)

### Specifics

**AWS Lambda Free Tier (Recommended for Development):**
- 1 million requests/month (free, permanent)
- 400,000 GB-seconds compute free
- Cold starts: 1-3 seconds for Python FastAPI (Mangum adapter)
- 10 GB container image limit
- Setup: Use Mangum adapter for ASGI → Lambda, API Gateway for HTTP routing

**Free Tier Capacity for RAG Chatbot:**
- 1M requests = ~33K requests/day
- With 1 GB memory: 400K seconds = ~111 hours/month runtime
- Average RAG query (2 seconds): 200K queries/month on free tier

**Railway ($5/month for Production):**
- No cold starts (persistent containers)
- 0.5 GB RAM instance: ~500 hours uptime (~20 days continuous)
- Good for FastAPI + WebSocket support
- Easy database integration with Qdrant and Neon

### Rationale
- **AWS Lambda**: Best value for development/prototyping (1M free requests), scales seamlessly to paid tier
- **Railway**: No cold starts for production chatbot (user experience requirement: <3s response time)
- **Avoid Render**: 50+ second cold starts (deal-breaker for RAG)
- **Avoid Vercel**: 250 MB bundle size limit problematic for ML dependencies, no persistent connections

### Alternatives Considered
- Render: True free tier, but 50s cold starts too slow for chatbot UX
- Fly.io: Fast cold starts (<2s), but no official free tier (~$5/month soft cap)
- Vercel Serverless: Size limits and serverless architecture don't suit RAG workloads with vector DB connections

---

## Summary of Actionable Recommendations

| Area | Recommendation | Justification |
|------|---------------|---------------|
| **Embeddings** | Qwen3 Embedding 8B via DeepInfra | $0.00000001/M tokens, MTEB #1 leaderboard |
| **OpenRouter Credits** | Purchase $10 credits | Unlocks 1,000 req/day limit permanently |
| **Vector DB** | Qdrant Cloud Free Tier | 1 GB (~1M vectors), sufficient for textbook |
| **Metadata DB** | Neon Serverless Postgres Free Tier | 0.5 GB, 100 CU-hours/month, scale to 0.25 CU |
| **Chunking** | Recursive, 400 tokens, 20% overlap | Balances context preservation with retrieval precision |
| **Deployment** | AWS Lambda (dev) → Railway (prod) | Free tier for prototyping, no cold starts for production |

---

## Implementation Impact

### Updated Technical Context
All NEEDS CLARIFICATION items resolved:

**OpenRouter API rate limits**:
- Free: 50 req/day, 20 RPM
- With $10 credits: 1,000 req/day, 20 RPM (permanent)

**Qdrant Cloud Free Tier limits**:
- 1 GB storage (~1M vectors at 768 dimensions)
- Single node, always free

**Neon Serverless Postgres Free Tier limits**:
- 0.5 GB storage, 100 CU-hours/month, 5 GB egress
- Scale to 0.25 CU for 400 hours/month uptime

### Next Steps (Phase 1)
1. Generate data-model.md with entities and relationships
2. Create API contracts in /contracts/
3. Write quickstart.md for local development setup
4. Update agent context file with technology stack
