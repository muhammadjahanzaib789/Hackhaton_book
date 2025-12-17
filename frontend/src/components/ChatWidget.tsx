/**
 * Chat Widget Main Component
 * Container for the entire chat interface with all subcomponents.
 */

import React, { useRef, useEffect, useState } from "react";
import { ChatMessage } from "./ChatMessage";
import { ChatInput } from "./ChatInput";
import { LoadingIndicator } from "./LoadingIndicator";
import { useChat } from "../hooks/useChat";
import { useSelection } from "../hooks/useSelection";
import "../styles/chat-widget.css";

interface ChatWidgetProps {
  apiUrl?: string;
}

export const ChatWidget: React.FC<ChatWidgetProps> = ({ apiUrl }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const { messages, isLoading, error, sendMessage, clearMessages, clearError } = useChat();
  const { text: selectedText, hasSelection } = useSelection();

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: "smooth" });
    }
  }, [messages]);

  const toggleWidget = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setIsMinimized(false);
    }
  };

  const toggleMinimize = () => {
    setIsMinimized(!isMinimized);
  };

  const handleClearChat = () => {
    if (window.confirm("Clear all messages?")) {
      clearMessages();
    }
  };

  return (
    <>
      {/* Floating Chat Button */}
      {!isOpen && (
        <button className="chat-widget-button" onClick={toggleWidget} aria-label="Open chat">
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z"
              fill="currentColor"
            />
            <path d="M6 9H18M6 13H15" stroke="white" strokeWidth="2" strokeLinecap="round" />
          </svg>
          {hasSelection && <span className="notification-badge">!</span>}
        </button>
      )}

      {/* Chat Widget Container */}
      {isOpen && (
        <div className={`chat-widget-container ${isMinimized ? "minimized" : ""}`}>
          {/* Header */}
          <div className="chat-widget-header">
            <div className="header-title">
              <span className="header-icon">💬</span>
              <span>Book Assistant</span>
            </div>
            <div className="header-actions">
              <button
                className="header-button"
                onClick={toggleMinimize}
                aria-label={isMinimized ? "Maximize" : "Minimize"}
              >
                {isMinimized ? "⬆" : "⬇"}
              </button>
              <button
                className="header-button"
                onClick={handleClearChat}
                aria-label="Clear chat"
                disabled={messages.length === 0}
              >
                🗑️
              </button>
              <button
                className="header-button close"
                onClick={toggleWidget}
                aria-label="Close chat"
              >
                ✕
              </button>
            </div>
          </div>

          {/* Chat Body */}
          {!isMinimized && (
            <>
              <div className="chat-widget-body">
                {messages.length === 0 ? (
                  <div className="empty-state">
                    <div className="empty-state-icon">📚</div>
                    <div className="empty-state-title">Ask me anything about the book!</div>
                    <div className="empty-state-description">
                      You can search the entire book or select text to ask specific questions.
                    </div>
                  </div>
                ) : (
                  <div className="messages-container">
                    {messages.map((message) => (
                      <ChatMessage key={message.id} message={message} />
                    ))}
                    {isLoading && <LoadingIndicator message="Searching the book..." />}
                    <div ref={messagesEndRef} />
                  </div>
                )}

                {error && (
                  <div className="error-banner">
                    <span className="error-icon">⚠️</span>
                    <span className="error-text">{error}</span>
                    <button className="error-close" onClick={clearError}>
                      ✕
                    </button>
                  </div>
                )}
              </div>

              {/* Chat Input */}
              <div className="chat-widget-footer">
                <ChatInput
                  onSendMessage={sendMessage}
                  isLoading={isLoading}
                  selectedText={selectedText}
                  hasSelection={hasSelection}
                />
              </div>
            </>
          )}
        </div>
      )}
    </>
  );
};
