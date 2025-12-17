/**
 * TypeScript interfaces for chat functionality.
 * Matches backend OpenAPI schemas from openapi.yaml.
 */

export enum SessionMode {
  FULL_BOOK = "full_book",
  SELECTED_TEXT = "selected_text",
}

export enum SessionStatus {
  ACTIVE = "active",
  ENDED = "ended",
  ABANDONED = "abandoned",
}

export interface SourceCitation {
  chunk_id: string;
  chapter_name: string;
  section_name?: string;
  page_number?: number;
  relevance_score: number;
  text_preview: string;
  excerpt?: string;
  link?: string;
}

export interface QueryRequest {
  session_id?: string;
  query: string;
  mode: SessionMode;
  selected_text?: string;
}

export interface QueryResponse {
  session_id: string;
  query_id: string;
  answer: string;
  sources: SourceCitation[];
  mode: SessionMode;
  processing_time_ms: number;
}

export interface ChatMessage {
  id: string;
  role: "user" | "assistant";
  content: string;
  sources?: SourceCitation[];
  timestamp: Date;
  mode?: SessionMode;
}

export interface ChatSession {
  session_id: string;
  mode: SessionMode;
  status: SessionStatus;
  started_at: Date;
  ended_at?: Date;
}

export interface ApiError {
  error: string;
  message: string;
  failed_files?: string[];
}

export interface HealthResponse {
  status: "healthy" | "degraded" | "unhealthy";
  timestamp: string;
  dependencies: {
    qdrant: DependencyStatus;
    neon_postgres: DependencyStatus;
    openrouter: DependencyStatus;
  };
}

export interface DependencyStatus {
  status: "up" | "down" | "degraded";
  latency_ms?: number;
  error?: string;
}
