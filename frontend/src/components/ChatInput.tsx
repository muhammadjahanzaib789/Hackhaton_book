/**
 * Chat Input Component
 * Input field for user queries with character count and submit button.
 */

import React, { useState, KeyboardEvent, ChangeEvent } from "react";
import { SessionMode } from "../types/chat";

interface ChatInputProps {
  onSendMessage: (query: string, mode: SessionMode, selectedText?: string) => void;
  isLoading: boolean;
  selectedText: string;
  hasSelection: boolean;
}

export const ChatInput: React.FC<ChatInputProps> = ({
  onSendMessage,
  isLoading,
  selectedText,
  hasSelection,
}) => {
  const [query, setQuery] = useState("");
  const [mode, setMode] = useState<SessionMode>(SessionMode.FULL_BOOK);

  const handleSubmit = () => {
    if (query.trim() && !isLoading) {
      onSendMessage(
        query,
        mode,
        mode === SessionMode.SELECTED_TEXT ? selectedText : undefined
      );
      setQuery("");
    }
  };

  const handleKeyPress = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  const handleChange = (e: ChangeEvent<HTMLTextAreaElement>) => {
    setQuery(e.target.value);
  };

  const charCount = query.length;
  const isValid = charCount >= 5 && charCount <= 500;

  // Auto-switch to selected_text mode if user has selection
  React.useEffect(() => {
    if (hasSelection && mode === SessionMode.FULL_BOOK) {
      setMode(SessionMode.SELECTED_TEXT);
    }
  }, [hasSelection, mode]);

  return (
    <div className="chat-input-container">
      {hasSelection && (
        <div className="selection-indicator">
          <span className="selection-icon">✓</span>
          <span className="selection-text">
            {selectedText.length} characters selected
          </span>
        </div>
      )}

      <div className="mode-selector">
        <label className="mode-option">
          <input
            type="radio"
            value={SessionMode.FULL_BOOK}
            checked={mode === SessionMode.FULL_BOOK}
            onChange={() => setMode(SessionMode.FULL_BOOK)}
            disabled={isLoading}
          />
          <span>Search Full Book</span>
        </label>
        <label className="mode-option">
          <input
            type="radio"
            value={SessionMode.SELECTED_TEXT}
            checked={mode === SessionMode.SELECTED_TEXT}
            onChange={() => setMode(SessionMode.SELECTED_TEXT)}
            disabled={isLoading || !hasSelection}
          />
          <span>
            Ask About Selection
            {!hasSelection && " (select text first)"}
          </span>
        </label>
      </div>

      <div className="input-wrapper">
        <textarea
          className="chat-input"
          value={query}
          onChange={handleChange}
          onKeyPress={handleKeyPress}
          placeholder={
            mode === SessionMode.SELECTED_TEXT
              ? "Ask a question about the selected text..."
              : "Ask a question about the book..."
          }
          disabled={isLoading}
          rows={3}
        />

        <div className="input-footer">
          <div className="char-count">
            <span className={charCount > 500 ? "error" : ""}>
              {charCount}/500
            </span>
            {charCount > 0 && charCount < 5 && (
              <span className="min-chars">(min 5 characters)</span>
            )}
          </div>

          <button
            className="send-button"
            onClick={handleSubmit}
            disabled={!isValid || isLoading}
          >
            {isLoading ? (
              <span className="loading-spinner">⏳</span>
            ) : (
              <span>Send</span>
            )}
          </button>
        </div>
      </div>
    </div>
  );
};
