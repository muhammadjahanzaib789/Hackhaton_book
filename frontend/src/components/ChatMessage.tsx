/**
 * Chat Message Component
 * Displays individual user or assistant messages with citations.
 */

import React from "react";
import { ChatMessage as ChatMessageType } from "../types/chat";
import { SourceCitation } from "./SourceCitation";

interface ChatMessageProps {
  message: ChatMessageType;
}

export const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const isUser = message.role === "user";

  return (
    <div className={`chat-message ${message.role}`}>
      <div className="message-bubble">
        <div className="message-content">{message.content}</div>

        {message.mode && (
          <div className="message-mode">
            <span className={`mode-badge ${message.mode}`}>
              {message.mode === "selected_text" ? "Selected Text" : "Full Book"}
            </span>
          </div>
        )}

        <div className="message-timestamp">
          {message.timestamp.toLocaleTimeString([], {
            hour: "2-digit",
            minute: "2-digit",
          })}
        </div>
      </div>

      {!isUser && message.sources && message.sources.length > 0 && (
        <div className="message-sources">
          <div className="sources-header">Sources:</div>
          <div className="sources-list">
            {message.sources.map((source, index) => (
              <SourceCitation key={source.chunk_id} citation={source} index={index} />
            ))}
          </div>
        </div>
      )}
    </div>
  );
};
