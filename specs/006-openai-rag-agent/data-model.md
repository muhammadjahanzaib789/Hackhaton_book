# Data Model: OpenAI RAG Agent

## AgentRequest
**Description**: User question input to the RAG agent system, containing the query text and any optional parameters.

**Fields**:
- `query_text` (string, required): The natural language question from the user
- `thread_id` (string, optional): OpenAI thread identifier for conversation continuity (if needed)
- `metadata` (object, optional): Additional context or parameters for the query

**Validation Rules**:
- `query_text` must be 1-500 characters
- `query_text` must not be empty or whitespace only

## AgentResponse
**Description**: Structured response from the RAG agent, containing the answer, source citations, and confidence indicators.

**Fields**:
- `answer` (string, required): The agent's response to the user's question
- `citations` (array[SourceCitation], required): List of sources used in the response
- `confidence_score` (number, optional): Confidence level of the response (0.0-1.0)
- `processing_time_ms` (number, required): Time taken to process the request in milliseconds
- `status` (string, required): Status of the request ("success", "partial", "error")
- `error_message` (string, optional): Error details if status is "error"

**Validation Rules**:
- `answer` must not be empty when status is "success"
- `confidence_score` must be between 0.0 and 1.0 if provided
- `processing_time_ms` must be a positive number

## SourceCitation
**Description**: Reference to specific book content used in the agent's response, including links to original source.

**Fields**:
- `chunk_id` (string, required): Unique identifier for the content chunk
- `text_preview` (string, required): Short preview of the cited content
- `url` (string, required): URL link to the original book content
- `section` (string, optional): Section name in the book
- `relevance_score` (number, required): Similarity score of the citation to the query (0.0-1.0)
- `page_reference` (string, optional): Page or location reference in the book

**Validation Rules**:
- `relevance_score` must be between 0.0 and 1.0
- `url` must be a valid URL format
- `text_preview` must be 1-500 characters

## RetrievalToolRequest
**Description**: Request parameters for the retrieval tool that the agent will invoke.

**Fields**:
- `query` (string, required): The query text to search for in the knowledge base
- `top_k` (number, optional): Number of results to return (default: 5, max: 10)
- `similarity_threshold` (number, optional): Minimum similarity score for results (default: 0.5)

**Validation Rules**:
- `query` must be 1-500 characters
- `top_k` must be between 1 and 10
- `similarity_threshold` must be between 0.0 and 1.0

## RetrievalToolResponse
**Description**: Response from the retrieval tool containing book content chunks.

**Fields**:
- `results` (array[RetrievedChunk], required): List of retrieved content chunks
- `query_latency` (number, required): Time taken for the retrieval in milliseconds

**Validation Rules**:
- `results` array must not be empty in successful responses
- `query_latency` must be a positive number

## RetrievedChunk
**Description**: Individual content chunk retrieved from the knowledge base.

**Fields**:
- `chunk_id` (string, required): Unique identifier for the chunk
- `text` (string, required): The actual content text
- `url` (string, required): URL of the source document
- `section` (string, optional): Section name in the document
- `chunk_index` (number, required): Position of this chunk in the original document
- `similarity_score` (number, required): Cosine similarity score (0.0-1.0)

**Validation Rules**:
- `similarity_score` must be between 0.0 and 1.0
- `chunk_index` must be a non-negative integer
- `text` must not be empty