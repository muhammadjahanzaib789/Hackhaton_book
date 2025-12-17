/**
 * Chat state management hook.
 * Handles message history, query submission, and session management.
 */

import { useState, useCallback } from "react";
import { v4 as uuidv4 } from "uuid";
import { apiClient } from "../services/api";
import {
  ChatMessage,
  QueryRequest,
  SessionMode,
} from "../types/chat";

export interface UseChatReturn {
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  sessionId: string | null;
  sendMessage: (query: string, mode: SessionMode, selectedText?: string) => Promise<void>;
  clearMessages: () => void;
  clearError: () => void;
}

export const useChat = (): UseChatReturn => {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string | null>(null);

  const sendMessage = useCallback(
    async (query: string, mode: SessionMode, selectedText?: string) => {
      if (!query.trim()) {
        setError("Please enter a question.");
        return;
      }

      if (query.trim().length < 5) {
        setError("Question must be at least 5 characters.");
        return;
      }

      if (query.trim().length > 500) {
        setError("Question must be less than 500 characters.");
        return;
      }

      // Validate selected text for selected_text mode
      if (mode === SessionMode.SELECTED_TEXT) {
        if (!selectedText || selectedText.length < 10) {
          setError("Please select at least 10 characters of text to ask about.");
          return;
        }
      }

      setIsLoading(true);
      setError(null);

      // Add user message to chat
      const userMessage: ChatMessage = {
        id: uuidv4(),
        role: "user",
        content: query,
        timestamp: new Date(),
        mode,
      };
      setMessages((prev) => [...prev, userMessage]);

      try {
        // Prepare request
        const request: QueryRequest = {
          session_id: sessionId || undefined,
          query: query.trim(),
          mode,
          selected_text: mode === SessionMode.SELECTED_TEXT ? selectedText : undefined,
        };

        // Call API
        const response = await apiClient.query(request);

        // Update session ID if new
        if (!sessionId) {
          setSessionId(response.session_id);
          // Set session ID in API client for rate limiting
          apiClient.setSessionId(response.session_id);
        }

        // Add assistant message to chat
        const assistantMessage: ChatMessage = {
          id: response.query_id,
          role: "assistant",
          content: response.answer,
          sources: response.sources,
          timestamp: new Date(),
          mode: response.mode,
        };
        setMessages((prev) => [...prev, assistantMessage]);
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : "An unexpected error occurred.";
        setError(errorMessage);

        // Add error message to chat
        const errorMsg: ChatMessage = {
          id: uuidv4(),
          role: "assistant",
          content: `Sorry, I encountered an error: ${errorMessage}`,
          timestamp: new Date(),
        };
        setMessages((prev) => [...prev, errorMsg]);
      } finally {
        setIsLoading(false);
      }
    },
    [sessionId]
  );

  const clearMessages = useCallback(() => {
    setMessages([]);
    setSessionId(null);
    setError(null);
  }, []);

  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    messages,
    isLoading,
    error,
    sessionId,
    sendMessage,
    clearMessages,
    clearError,
  };
};
