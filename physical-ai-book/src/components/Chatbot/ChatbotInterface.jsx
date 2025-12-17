import React from 'react';
import useChatbot from '@site/src/hooks/useChatbot';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import './chatbot.css';

// Component to be embedded in Docusaurus pages
const ChatbotInterface = ({ position = 'inline' }) => {
  const {
    messages,
    isLoading,
    currentMode,
    selectedText,
    sendMessage,
    checkHealth
  } = useChatbot();

  const handleSendMessage = async (message, mode, selectedTextValue = null) => {
    // Use the passed selectedTextValue or the currently stored selectedText
    const textToUse = selectedTextValue || selectedText;
    await sendMessage(message, mode, textToUse);
  };

  // Determine CSS class based on position prop
  const containerClass = position === 'floating'
    ? 'chatbot-interface floating'
    : 'chatbot-interface';

  return (
    <div className={containerClass}>
      <div className="chat-header">
        <h3>RAG Chatbot</h3>
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your RAG chatbot. Ask me questions about the book content.</p>
            <p>You can ask questions about the full book or select specific text to limit the scope of my responses.</p>
          </div>
        ) : (
          messages.map(message => (
            <ChatMessage
              key={message.id}
              message={message}
            />
          ))
        )}

        {isLoading && (
          <div className="loading-indicator">
            <div className="typing-indicator">
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
      </div>

      <ChatInput
        onSendMessage={handleSendMessage}
        isLoading={isLoading}
        currentMode={currentMode}
      />
    </div>
  );
};

export default ChatbotInterface;