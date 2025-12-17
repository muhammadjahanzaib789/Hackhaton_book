import React, { useState } from 'react';
import { FaPaperPlane, FaMousePointer } from 'react-icons/fa';

const ChatInput = ({ onSendMessage, isLoading, currentMode }) => {
  const [inputText, setInputText] = useState('');
  const [mode, setMode] = useState(currentMode || 'full_book'); // 'full_book' or 'selected_text'

  // Update mode when the prop changes
  React.useEffect(() => {
    if (currentMode) {
      setMode(currentMode);
    }
  }, [currentMode]);

  const handleSubmit = (e) => {
    e.preventDefault();

    if (!inputText.trim() || isLoading) return;

    let selectedText = null;
    if (mode === 'selected_text') {
      selectedText = window.getSelection ? window.getSelection().toString().trim() : '';
      if (!selectedText) {
        alert('Please select text on the page before using selected text mode.');
        return;
      }
    }

    onSendMessage(inputText, mode, selectedText);
    setInputText('');
  };

  return (
    <form className="chat-input-form" onSubmit={handleSubmit}>
      <div className="input-controls">
        <select
          value={mode}
          onChange={(e) => setMode(e.target.value)}
          className="mode-selector"
          disabled={isLoading}
        >
          <option value="full_book">Full Book</option>
          <option value="selected_text">Selected Text</option>
        </select>

        {mode === 'selected_text' && (
          <div className="mode-indicator selected-text">
            <FaMousePointer className="indicator-icon" />
            <span className="indicator-text">Select text on page</span>
          </div>
        )}
      </div>

      <div className="input-container">
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          placeholder="Ask a question about the book..."
          className="chat-input"
          disabled={isLoading}
          rows="1"
        />
        <button
          type="submit"
          className="send-button"
          disabled={isLoading || !inputText.trim()}
        >
          <FaPaperPlane />
        </button>
      </div>
    </form>
  );
};

export default ChatInput;