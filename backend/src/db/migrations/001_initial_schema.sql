-- Initial database schema for RAG Chatbot
-- Based on data-model.md specification

-- Enable UUID extension if not already enabled
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Table: book_content_chunks
-- Stores metadata for book content chunks (vectors stored in Qdrant)
CREATE TABLE IF NOT EXISTS book_content_chunks (
    chunk_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    text TEXT NOT NULL CHECK (char_length(text) BETWEEN 100 AND 2000),
    chapter_name VARCHAR(200) NOT NULL,
    section_name VARCHAR(200),
    page_number INTEGER CHECK (page_number >= 1),
    document_path VARCHAR(500) NOT NULL,
    chunk_index INTEGER NOT NULL CHECK (chunk_index >= 0),
    token_count INTEGER NOT NULL CHECK (token_count BETWEEN 100 AND 500),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB,
    CONSTRAINT unique_chunk_per_doc UNIQUE (document_path, chunk_index)
);

-- Table: chat_sessions
-- Tracks user interaction sessions
CREATE TABLE IF NOT EXISTS chat_sessions (
    session_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id VARCHAR(100),
    started_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    last_activity_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    mode VARCHAR(20) NOT NULL DEFAULT 'full_book' CHECK (mode IN ('full_book', 'selected_text')),
    query_count INTEGER NOT NULL DEFAULT 0 CHECK (query_count >= 0),
    status VARCHAR(20) NOT NULL DEFAULT 'active' CHECK (status IN ('active', 'ended', 'abandoned')),
    CONSTRAINT valid_activity_time CHECK (last_activity_at >= started_at)
);

-- Table: queries
-- Stores user questions
CREATE TABLE IF NOT EXISTS queries (
    query_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    session_id UUID NOT NULL REFERENCES chat_sessions(session_id) ON DELETE CASCADE,
    query_text TEXT NOT NULL CHECK (char_length(query_text) BETWEEN 5 AND 500),
    selected_text TEXT CHECK (selected_text IS NULL OR char_length(selected_text) >= 10),
    mode VARCHAR(20) NOT NULL CHECK (mode IN ('full_book', 'selected_text')),
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    processing_time_ms INTEGER CHECK (processing_time_ms >= 0),
    error VARCHAR(500),
    CONSTRAINT valid_selected_text_mode CHECK (
        (mode = 'selected_text' AND selected_text IS NOT NULL) OR
        (mode = 'full_book' AND selected_text IS NULL)
    )
);

-- Table: responses
-- Stores chatbot answers
CREATE TABLE IF NOT EXISTS responses (
    response_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    query_id UUID NOT NULL UNIQUE REFERENCES queries(query_id) ON DELETE CASCADE,
    response_text TEXT NOT NULL CHECK (char_length(response_text) BETWEEN 50 AND 2000),
    confidence_score REAL CHECK (confidence_score BETWEEN 0.0 AND 1.0),
    model_used VARCHAR(100) NOT NULL,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    token_count INTEGER CHECK (token_count >= 0)
);

-- Table: source_citations
-- References to book content used in responses
CREATE TABLE IF NOT EXISTS source_citations (
    citation_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    response_id UUID NOT NULL REFERENCES responses(response_id) ON DELETE CASCADE,
    chunk_id UUID NOT NULL REFERENCES book_content_chunks(chunk_id) ON DELETE CASCADE,
    chapter_name VARCHAR(200) NOT NULL,
    section_name VARCHAR(200),
    page_number INTEGER CHECK (page_number >= 1),
    relevance_score REAL NOT NULL CHECK (relevance_score BETWEEN 0.0 AND 1.0),
    excerpt VARCHAR(500),
    link VARCHAR(500)
);

-- Indexes for performance optimization

-- book_content_chunks indexes
CREATE INDEX idx_chunks_chapter ON book_content_chunks(chapter_name);
CREATE INDEX idx_chunks_document_path ON book_content_chunks(document_path);
CREATE INDEX idx_chunks_created_at ON book_content_chunks(created_at DESC);

-- chat_sessions indexes
CREATE INDEX idx_sessions_started_at ON chat_sessions(started_at DESC);
CREATE INDEX idx_sessions_status ON chat_sessions(status);
CREATE INDEX idx_sessions_user_id ON chat_sessions(user_id);

-- queries indexes
CREATE INDEX idx_queries_session_id ON queries(session_id);
CREATE INDEX idx_queries_timestamp ON queries(timestamp DESC);
CREATE INDEX idx_queries_mode ON queries(mode);

-- responses indexes
CREATE INDEX idx_responses_query_id ON responses(query_id);
CREATE INDEX idx_responses_timestamp ON responses(timestamp DESC);

-- source_citations indexes
CREATE INDEX idx_citations_response_id ON source_citations(response_id);
CREATE INDEX idx_citations_chunk_id ON source_citations(chunk_id);
CREATE INDEX idx_citations_relevance_score ON source_citations(relevance_score DESC);

-- Comments for documentation
COMMENT ON TABLE book_content_chunks IS 'Metadata for book content chunks (vectors in Qdrant)';
COMMENT ON TABLE chat_sessions IS 'User interaction sessions';
COMMENT ON TABLE queries IS 'User questions submitted to chatbot';
COMMENT ON TABLE responses IS 'Chatbot-generated answers';
COMMENT ON TABLE source_citations IS 'References to book content used in answers';

COMMENT ON COLUMN queries.mode IS 'Query mode: full_book or selected_text';
COMMENT ON COLUMN queries.selected_text IS 'User-selected text (only for selected_text mode)';
COMMENT ON COLUMN responses.confidence_score IS 'LLM confidence score (0.0-1.0)';
COMMENT ON COLUMN source_citations.relevance_score IS 'Vector similarity score (0.0-1.0)';
