# Feature Specification: Integrated RAG Chatbot for Published Book

**Feature Branch**: `001-integrated-rag-chatbot`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "Published book ke andar **Integrated RAG (Retrieval-Augmented Generation) Chatbot** embed karna jo: Book ke **full content** par questions answer kare, **User-selected text only** par bhi strictly answer kare, **OpenRouter API** use kare (LLMs ke liye), **Qwen embeddings** use kare (semantic search ke liye), **FastAPI** backend ho, **Neon Serverless Postgres** metadata & chat logs ke liye, **Qdrant Cloud (Free Tier)** vector database ke liye, Web UI se easily embed ho (Docusaurus / static book site)"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Entire Book Content (Priority: P1)

As a reader, I want to ask questions about any topic covered in the book and receive accurate, source-referenced answers so that I can quickly find information without manually searching through chapters.

**Why this priority**: This is the core value proposition of a RAG chatbot - enabling semantic search and question-answering across the entire book content. Without this, the feature has no fundamental purpose.

**Independent Test**: Can be fully tested by submitting a question about any chapter's content through the chat interface and verifying that the answer is accurate and includes source citations (chapter/page references).

**Acceptance Scenarios**:

1. **Given** I am viewing the published book website, **When** I type "What are the key components of a robotic control system?" in the chat interface, **Then** I receive an answer synthesized from relevant book sections with citations showing chapter and page numbers
2. **Given** I ask a question about a topic spread across multiple chapters, **When** the system processes my query, **Then** I receive a comprehensive answer that references all relevant sections with proper citations
3. **Given** I ask about a topic not covered in the book, **When** the system searches the content, **Then** I receive a clear message stating the topic is not found in the book rather than hallucinating an answer
4. **Given** I am on any page of the book, **When** I open the chatbot, **Then** the chat interface is accessible and functional without requiring page reload or navigation

---

### User Story 2 - Ask Questions About Selected Text Only (Priority: P2)

As a reader, I want to highlight a specific passage or paragraph and ask questions strictly about that selected text so that I can get clarifications on specific concepts without the chatbot searching the entire book.

**Why this priority**: This provides focused, context-specific help for readers who need clarification on particular sections they're currently reading, enhancing the learning experience and preventing irrelevant answers from other parts of the book.

**Independent Test**: Can be tested by selecting a paragraph, asking a question, and verifying that the answer only references the selected text and does not include information from other chapters.

**Acceptance Scenarios**:

1. **Given** I have selected a paragraph about neural networks, **When** I ask "Explain this in simpler terms", **Then** the chatbot provides a simplified explanation based only on the selected text without referencing other chapters
2. **Given** I have selected text and asked a question, **When** I clear the selection, **Then** the chatbot switches back to full-book search mode for subsequent questions
3. **Given** I have selected a code example, **When** I ask "What does this code do?", **Then** the chatbot explains only the selected code snippet without pulling in explanations from other code examples in the book
4. **Given** I select text that contains technical terminology, **When** I ask "Define the terms here", **Then** the chatbot provides definitions based on how those terms are used in the selected passage

---

### User Story 3 - View Source Citations and Navigate to Referenced Sections (Priority: P3)

As a reader, I want to see exactly which chapters, sections, or pages were used to generate each answer so that I can verify information and read the original context in detail.

**Why this priority**: This builds trust in the chatbot's answers and allows readers to dive deeper into topics of interest. While valuable, it's not essential for the basic Q&A functionality to work.

**Independent Test**: Can be tested by asking any question and verifying that the answer includes clickable links or references that navigate to the exact location in the book where the information was found.

**Acceptance Scenarios**:

1. **Given** I receive an answer from the chatbot, **When** I review the response, **Then** I see citations formatted as "Chapter X: Section Name" or "Page Y" for each referenced source
2. **Given** the answer cites multiple sources, **When** I click on a citation link, **Then** I am navigated to that specific chapter/section in the book
3. **Given** an answer references a specific diagram or code example, **When** I view the citation, **Then** the citation specifically mentions the figure/code listing number
4. **Given** I am viewing a long answer with multiple citations, **When** I hover over or click a citation, **Then** I can preview a snippet of the original text without leaving the chat interface

---

### User Story 4 - Fast Response Times for Interactive Learning (Priority: P2)

As a reader, I want to receive answers within 2-3 seconds so that the chat feels interactive and doesn't disrupt my learning flow.

**Why this priority**: Response speed directly impacts user experience and whether readers will continue using the chatbot. Slow responses lead to frustration and abandonment.

**Independent Test**: Can be tested by submitting various questions (simple and complex) and measuring response times using browser developer tools or automated testing.

**Acceptance Scenarios**:

1. **Given** I ask a straightforward factual question, **When** I submit the query, **Then** I receive an answer in under 2 seconds
2. **Given** I ask a complex question requiring synthesis from multiple chapters, **When** the system processes it, **Then** I receive an answer in under 3 seconds
3. **Given** the system is experiencing high load, **When** response time exceeds 5 seconds, **Then** I see a loading indicator or progress message to manage expectations
4. **Given** the backend service is temporarily unavailable, **When** I submit a question, **Then** I receive a clear error message within 2 seconds rather than hanging indefinitely

---

### Edge Cases

- **What happens when a user submits a very long question (>500 words)?** System should either process it normally (if within token limits) or return a friendly error asking the user to shorten the question.
- **What happens when selected text is very short (1-2 words) and insufficient for context?** System should notify the user that the selection is too brief to provide meaningful answers and suggest selecting more context.
- **What happens when the book content is updated or new chapters are added?** System should have a mechanism to re-index the content without disrupting active user sessions.
- **What happens when network connectivity is lost during a query?** System should display a clear error message and allow the user to retry the request.
- **What happens if a user asks questions in a language different from the book's language?** System should detect non-English queries and respond with a polite message requesting the user to ask in English, as the book content and chatbot are optimized for English-language interactions only.
- **What happens when multiple users select the same passage simultaneously?** Each user's session should be independent and not interfere with others.
- **What happens if the embedding or LLM service is temporarily down?** System should fail gracefully with a clear message explaining the service is temporarily unavailable and suggesting the user try again later.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST retrieve and index all textual content from the published book (all chapters, sections, and subsections) for semantic search
- **FR-002**: System MUST support two distinct query modes: full-book search and selected-text-only search
- **FR-003**: System MUST generate vector embeddings for book content using Qwen embeddings to enable semantic retrieval
- **FR-004**: System MUST use OpenRouter API to generate natural language answers from retrieved content
- **FR-005**: System MUST include source citations (chapter name, section name, or page identifier) in every answer
- **FR-006**: System MUST respond to queries within 3 seconds under normal load conditions
- **FR-007**: System MUST persist chat interaction logs (query, response, timestamp, mode) for analytics and debugging
- **FR-008**: System MUST handle cases where no relevant content is found by informing the user rather than hallucinating
- **FR-009**: System MUST allow users to switch between full-book and selected-text modes seamlessly
- **FR-010**: System MUST sanitize and validate user input to prevent injection attacks or malformed queries
- **FR-011**: System MUST expose API endpoints that can be consumed by any web-based frontend (Docusaurus, static sites, etc.)
- **FR-012**: System MUST support concurrent user sessions without cross-contamination of chat contexts
- **FR-013**: System MUST store vector embeddings in Qdrant Cloud vector database for efficient similarity search
- **FR-014**: System MUST store metadata (chapter mappings, embedding IDs, chat logs) in Neon Serverless Postgres
- **FR-015**: System MUST secure API keys and credentials using environment variables (never exposed to client)
- **FR-016**: Frontend MUST be embeddable as a widget or component in the existing book website without requiring major structural changes

### Key Entities

- **Book Content Chunk**: Represents a segment of the book (paragraph, section, or page) with associated text, embedding vector, and metadata (chapter, page, section ID)
- **Chat Session**: Represents a user's interaction session with unique session ID, timestamp, and mode (full-book or selected-text)
- **Query**: Represents a user's question with text, timestamp, session ID, mode, and optional selected text context
- **Response**: Represents the chatbot's answer with generated text, source citations, retrieved chunks, timestamp, and confidence score
- **Source Citation**: Represents a reference to book content with chapter name, section name, page number, and optionally a link/anchor to the exact location
- **Embedding**: Represents a vector representation of book content chunks stored in Qdrant with chunk ID and metadata

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can ask questions about book content and receive accurate, source-referenced answers in under 3 seconds for 95% of queries
- **SC-002**: Selected-text mode restricts answers to only the highlighted passage with zero cross-contamination from other book sections
- **SC-003**: Chatbot achieves at least 85% answer accuracy (measured by human evaluation of whether answers correctly address the question using book content)
- **SC-004**: System successfully handles 100 concurrent users without response time degradation beyond 5 seconds
- **SC-005**: Every chatbot response includes at least one verifiable source citation that links back to the correct chapter/section
- **SC-006**: Zero API keys or credentials are exposed to the client-side code (verified through security audit)
- **SC-007**: Chat interface successfully embeds into Docusaurus-based book website without breaking existing navigation or styling
- **SC-008**: System operates within free tier limits of Qdrant Cloud and Neon Serverless Postgres for initial deployment (up to 10,000 queries/month)

## Assumptions

- The book content is available in a machine-readable format (Markdown, HTML, or structured text) that can be parsed and chunked for embedding
- The book's structure includes clear chapter and section markers that can be used for citation generation
- Users will primarily ask questions in English (the book's language); multilingual support is deferred to future iterations
- The Docusaurus website has a mechanism to inject custom React components or JavaScript widgets for chatbot integration
- OpenRouter API and Qwen embeddings have sufficient quota/rate limits to handle expected user traffic
- Chat sessions are stateless on the frontend; all state is managed by the backend API

## Out of Scope

- Real-time collaborative features (multiple users discussing the same question)
- Voice-based queries or text-to-speech responses
- User accounts, authentication, or personalized chat history accessible across sessions
- Integration with external knowledge sources beyond the book content
- Mobile native apps (focus is web-based chatbot embeddable in the book website)
- Advanced analytics dashboard for tracking user queries and popular topics (basic logging only)
- Custom fine-tuning of LLMs specifically for this book (relying on general-purpose models via OpenRouter)
