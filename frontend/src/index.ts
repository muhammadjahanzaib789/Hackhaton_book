/**
 * Main entry point for the chat widget.
 * Exports all public components and types.
 */

export { ChatWidget } from "./components/ChatWidget";
export { ChatMessage } from "./components/ChatMessage";
export { ChatInput } from "./components/ChatInput";
export { SourceCitation } from "./components/SourceCitation";
export { LoadingIndicator } from "./components/LoadingIndicator";

export { useChat } from "./hooks/useChat";
export { useSelection } from "./hooks/useSelection";

export { apiClient } from "./services/api";

export type {
  ChatMessage as ChatMessageType,
  QueryRequest,
  QueryResponse,
  SourceCitation as SourceCitationType,
  SessionMode,
  SessionStatus,
} from "./types/chat";
