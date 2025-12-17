---
id: 002
title: RAG Chatbot Planning Workflow
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-integrated-rag-chatbot
branch: 001-integrated-rag-chatbot
user: Sheheryar
command: /sp.plan
labels: ["rag", "chatbot", "planning", "architecture", "openrouter", "qdrant", "neon-postgres"]
links:
  spec: specs/001-integrated-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-integrated-rag-chatbot/plan.md
  - specs/001-integrated-rag-chatbot/research.md
  - specs/001-integrated-rag-chatbot/data-model.md
  - specs/001-integrated-rag-chatbot/contracts/openapi.yaml
  - specs/001-integrated-rag-chatbot/contracts/README.md
  - specs/001-integrated-rag-chatbot/quickstart.md
  - CLAUDE.md
tests:
  - Constitution Check evaluation (pre-design)
  - Constitution Check re-evaluation (post-design)
---

## Prompt

## /sp.plan

### Architecture Overview

[ Book Website ]
|
| (User Question + Optional Selected Text)
v
[ Chat UI Widget ]
|
v
[ FastAPI Backend ]
|
|-- Embedding (Qwen via OpenRouter)
|-- Retrieval (Qdrant)
|-- Metadata (Neon Postgres)
v
[ LLM via OpenRouter ]
|
v
[ Answer + Sources ]

### Data Flow
1. Book content → chunking → embeddings → Qdrant
2. User question:
   - If **selected text exists** → only that text embedded & queried
   - Else → full book vector search
3. Retrieved chunks → prompt construction
4. OpenRouter LLM generates answer
5. Answer returned with references

## Response snapshot

Executed complete planning workflow for the Integrated RAG Chatbot feature (001-integrated-rag-chatbot). The workflow consisted of:

**Phase 0: Research & Unknowns Resolution**
- Researched OpenRouter API rate limits and Qwen3 Embedding 8B models
- Investigated Qdrant Cloud Free Tier limits (~1M vectors, 1GB storage)
- Analyzed Neon Serverless Postgres Free Tier (0.5GB, 100 CU-hours/month)
- Determined optimal RAG chunking strategy (400 tokens, 20% overlap, recursive chunking)
- Compared FastAPI deployment options (AWS Lambda for dev, Railway for prod)
- Generated `research.md` with all findings and actionable recommendations

**Phase 1: Design & Contracts**
- Created complete data model with 5 entities:
  - BookContentChunk (stored in Qdrant + Neon Postgres)
  - ChatSession (conversation tracking)
  - Query (user questions)
  - Response (LLM-generated answers)
  - SourceCitation (references to book content)
- Generated OpenAPI 3.0 contract with 3 endpoints:
  - GET /health (health check)
  - POST /query (submit questions)
  - POST /index (admin: re-index content)
- Wrote comprehensive quickstart.md with:
  - Prerequisites and setup instructions
  - Backend setup (Python, FastAPI, Qdrant, Neon)
  - Frontend setup (React/TypeScript chat widget)
  - Docusaurus integration
  - Testing commands
  - Troubleshooting guide
- Updated agent context file (CLAUDE.md) with technology stack

**Constitution Check**
- Pre-design: APPROVED with conditional passes (Principles II & III not applicable for infrastructure)
- Post-design: All principles satisfied, no new concerns identified

**Technical Context**
- Backend: Python 3.11+, FastAPI, OpenRouter SDK, Qdrant Client, psycopg3
- Frontend: React 18, TypeScript, Docusaurus 3.9+
- Storage: Qdrant Cloud (vectors), Neon Serverless Postgres (metadata)
- Performance: <3s response time for 95% queries, 100 concurrent users
- Deployment: AWS Lambda (dev), Railway (prod)

**Key Decisions**
1. Qwen3 Embedding 8B via DeepInfra for embeddings ($0.00000001/M tokens)
2. 400-token chunks with 20% overlap using recursive chunking
3. Hybrid storage: Qdrant for vectors, Neon for metadata
4. OpenRouter with $10 credits for 1,000 req/day limit

**Project Structure**
- Backend: FastAPI with services layer (embeddings, retrieval, LLM, content_indexer, chat_logger)
- Frontend: React chat widget components with hooks (useChat, useSelection)
- Integration: Docusaurus theme customization (Root.tsx)

**Ready for Phase 2**: Tasks generation (`/sp.tasks`) can now proceed with complete design artifacts.

## Outcome

- ✅ Impact: Complete planning phase for RAG chatbot feature. Generated 7 design artifacts (plan.md, research.md, data-model.md, OpenAPI spec, quickstart.md, contracts README). All NEEDS CLARIFICATION items resolved with concrete decisions.
- 🧪 Tests: No code tests yet (planning phase). Validated Constitution Check (2 evaluations: pre-design and post-design).
- 📁 Files: Created/modified 7 files in specs/001-integrated-rag-chatbot/ directory and updated CLAUDE.md
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation tasks based on data model and API contracts
- 🧠 Reflection: Planning workflow successfully integrated user-provided architecture diagram with research phase. Research agent provided comprehensive findings on all unknowns (API limits, chunking strategies, deployment options). Data model and API contracts are complete and ready for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None. Research agent successfully resolved all NEEDS CLARIFICATION items. PowerShell scripts worked correctly for setup and agent context update.
- Graders run and results (PASS/FAIL): Constitution Check - PASS (pre-design and post-design). All principles satisfied or conditionally approved with justification.
- Prompt variant (if applicable): Standard /sp.plan workflow with user-provided architecture diagram
- Next experiment (smallest change to try): Proceed to tasks generation phase. Consider adding ADR for architectural decisions (Qwen vs alternatives, chunking strategy, deployment choice) if significant enough during tasks phase.
