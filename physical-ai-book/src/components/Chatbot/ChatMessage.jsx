import React from 'react';
import { FaRobot, FaUser, FaExclamationTriangle } from 'react-icons/fa';
import CitationRenderer from './CitationRenderer';

const ChatMessage = ({ message }) => {
  const isUser = message.sender === 'user';
  const isBot = message.sender === 'bot';
  const isSystem = message.sender === 'system';

  return (
    <div className={`message ${isUser ? 'user-message' : ''} ${isBot ? 'bot-message' : ''} ${isSystem ? 'system-message' : ''}`}>
      <div className="message-header">
        {isUser && <FaUser className="user-icon" />}
        {isBot && !message.isError && <FaRobot className="bot-icon" />}
        {isBot && message.isError && <FaExclamationTriangle className="error-icon" />}
        <span className="sender">{isUser ? 'You' : isBot ? 'Chatbot' : 'System'}</span>
      </div>
      <div className="message-content">
        <p>{message.text}</p>
        
        {message.citations && message.citations.length > 0 && (
          <CitationRenderer citations={message.citations} />
        )}
      </div>
      {message.timestamp && (
        <div className="message-timestamp">
          {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </div>
      )}
    </div>
  );
};

export default ChatMessage;