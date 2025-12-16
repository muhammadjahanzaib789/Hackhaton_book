-- Migration 002: Add citation_order column to source_citations table
-- This enables ordering citations by relevance in the response

ALTER TABLE source_citations
ADD COLUMN IF NOT EXISTS citation_order INTEGER;

-- Add index for faster ordering
CREATE INDEX IF NOT EXISTS idx_citations_order ON source_citations(response_id, citation_order);

-- Comment
COMMENT ON COLUMN source_citations.citation_order IS 'Order of citation in response (1-indexed, ordered by relevance)';
