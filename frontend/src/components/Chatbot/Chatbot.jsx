import React, { useState, useRef, useEffect } from 'react';
import './Chatbot.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! 👋 I'm your AI assistant. How can I help you today?", sender: 'bot', timestamp: new Date() }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef(null);

  // Sample responses for the chatbot
  const responses = {
    greetings: [
      "Hello! How can I assist you today?",
      "Hi there! What can I do for you?",
      "Greetings! How may I help you?"
    ],
    thanks: [
      "You're welcome!",
      "Happy to help!",
      "Anytime! Is there anything else I can assist with?"
    ],
    help: [
      "I can help you with general inquiries, provide information, or guide you through common tasks.",
      "I'm here to answer your questions and provide assistance.",
      "Feel free to ask me anything you need help with!"
    ],
    goodbye: [
      "Goodbye! Feel free to come back if you have more questions.",
      "See you later! Have a great day!",
      "Take care! I'm always here if you need assistance."
    ],
    default: [
      "I'm not sure I understand. Could you rephrase that?",
      "Can you provide more details about your question?",
      "I'm still learning. Could you try asking in a different way?",
      "Interesting question! Unfortunately, I don't have enough information to provide a detailed answer right now."
    ]
  };

  // Common keywords and phrases to detect user intent
  const keywordPatterns = [
    { keywords: ['hello', 'hi', 'hey', 'greetings'], responses: responses.greetings },
    { keywords: ['thank', 'thanks', 'thank you', 'appreciate'], responses: responses.thanks },
    { keywords: ['help', 'assist', 'support', 'guidance'], responses: responses.help },
    { keywords: ['bye', 'goodbye', 'see you', 'farewell'], responses: responses.goodbye }
  ];

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  // Effect to scroll when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to handle sending a message
  const handleSendMessage = (e) => {
    e.preventDefault();
    
    if (inputValue.trim()) {
      // Add user message
      const userMessage = {
        id: Date.now(),
        text: inputValue,
        sender: 'user',
        timestamp: new Date()
      };
      
      setMessages(prev => [...prev, userMessage]);
      setInputValue('');
      
      // Show typing indicator
      setIsTyping(true);
      
      // Simulate bot response after delay
      setTimeout(() => {
        // Convert message to lowercase for easier matching
        const lowerMsg = inputValue.toLowerCase();
        
        // Check for matches in our patterns
        let foundResponse = false;
        
        for (const pattern of keywordPatterns) {
          if (pattern.keywords.some(keyword => lowerMsg.includes(keyword))) {
            const randomResponse = pattern.responses[Math.floor(Math.random() * pattern.responses.length)];
            const botMessage = {
              id: Date.now() + 1,
              text: randomResponse,
              sender: 'bot',
              timestamp: new Date()
            };
            setMessages(prev => [...prev, botMessage]);
            foundResponse = true;
            break;
          }
        }
        
        // If no specific pattern matched, use default response
        if (!foundResponse) {
          // Simple AI simulation - check for question words
          if (lowerMsg.includes('?') || lowerMsg.startsWith('what') || 
              lowerMsg.startsWith('how') || lowerMsg.startsWith('why') ||
              lowerMsg.startsWith('when') || lowerMsg.startsWith('where')) {
              
            // Generate slightly more thoughtful responses for questions
            const questionResponses = [
              "That's an interesting question. Based on what I know, ",
              "I'd be happy to address that. ",
              "Let me think about that for a moment. ",
              "Great question! "
            ];
            const followUps = [
              "I recommend checking our documentation or reaching out to our support team for more details.",
              "Could you provide more context so I can better assist you?",
              "I can help you find the information you need.",
              "I suggest exploring our FAQ section for related information."
            ];
            
            const randomQuestionResponse = questionResponses[Math.floor(Math.random() * questionResponses.length)];
            const randomFollowUp = followUps[Math.floor(Math.random() * followUps.length)];
            const botMessage = {
              id: Date.now() + 1,
              text: randomQuestionResponse + randomFollowUp,
              sender: 'bot',
              timestamp: new Date()
            };
            setMessages(prev => [...prev, botMessage]);
          } else {
            // Use default responses for general messages
            const randomResponse = responses.default[Math.floor(Math.random() * responses.default.length)];
            const botMessage = {
              id: Date.now() + 1,
              text: randomResponse,
              sender: 'bot',
              timestamp: new Date()
            };
            setMessages(prev => [...prev, botMessage]);
          }
        }
        
        setIsTyping(false);
      }, 1000 + Math.random() * 1000); // Random delay between 1-2 seconds
    }
  };

  // Toggle chatbot open/close
  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  // Format time for display
  const formatTime = (date) => {
    return date.getHours().toString().padStart(2, '0') + ':' + 
           date.getMinutes().toString().padStart(2, '0');
  };

  return (
    <>
      {/* Floating chatbot icon */}
      {!isOpen && (
        <div className="chatbot-fab" onClick={toggleChatbot}>
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H17.13L16.09 18.05C15.88 18.26 15.61 18.4 15.32 18.45C15.03 18.5 14.73 18.46 14.47 18.34C14.21 18.22 13.99 18.02 13.84 17.77C13.69 17.52 13.62 17.24 13.64 16.95L14.5 12C14.5 10.4087 13.8679 8.88258 12.7426 7.75736C11.6174 6.63214 10.0913 6 8.5 6C6.9087 6 5.38258 6.63214 4.25736 7.75736C3.13214 8.88258 2.5 10.4087 2.5 12C2.5 13.5913 3.13214 15.1174 4.25736 16.2426C5.38258 17.3679 6.9087 18 8.5 18H9.5C9.76522 18 10.0196 17.8946 10.2071 17.7071C10.3946 17.5196 10.5 17.2652 10.5 17C10.5 16.7348 10.3946 16.4804 10.2071 16.2929C10.0196 16.1054 9.76522 16 9.5 16H8.5C7.43913 16 6.42172 15.5786 5.67157 14.8284C4.92143 14.0783 4.5 13.0609 4.5 12C4.5 10.9391 4.92143 9.92172 5.67157 9.17157C6.42172 8.42143 7.43913 8 8.5 8C9.56087 8 10.5783 8.42143 11.3284 9.17157C12.0786 9.92172 12.5 10.9391 12.5 12C12.5 12.1904 12.4846 12.379 12.4548 12.5632C12.425 12.7475 12.3812 12.9262 12.3244 13.0972C12.2676 13.2682 12.1983 13.4303 12.1177 13.5817C12.0371 13.7331 11.9459 13.8728 11.8459 14H15.27L17.95 16.68C18.06 16.79 18.2 16.86 18.35 16.88C18.5 16.9 18.65 16.87 18.78 16.79C18.91 16.71 19.01 16.59 19.07 16.45C19.13 16.31 19.14 16.15 19.1 16L20 11.05C20.02 10.76 19.95 10.48 19.79 10.23C19.63 9.98 19.41 9.78 19.15 9.66C18.89 9.54 18.6 9.5 18.31 9.57C18.02 9.64 17.76 9.79 17.57 10L16.5 11C16.28 11.2 16.01 11.35 15.71 11.42C15.41 11.49 15.1 11.48 14.81 11.39L13.95 9.05C13.85 8.78 13.82 8.49 13.86 8.2C13.9 7.91 14.02 7.64 14.2 7.42C14.38 7.2 14.61 7.03 14.88 6.93C15.15 6.83 15.44 6.81 15.72 6.87L17.88 7.29C18.1 7.33 18.3 7.42 18.47 7.55C18.64 7.68 18.77 7.85 18.85 8.04C18.93 8.23 18.96 8.44 18.94 8.65C18.92 8.86 18.85 9.06 18.74 9.23L15.77 12.16C15.84 12.13 15.91 12.11 15.99 12.09C16.07 12.07 16.14 12.05 16.22 12.04C16.3 12.03 16.37 12.02 16.45 12.01C16.53 12 16.61 12 16.69 12.01C16.86 12.02 17.02 12.06 17.17 12.13C17.32 12.2 17.46 12.3 17.57 12.42C17.68 12.54 17.77 12.68 17.83 12.83C17.89 12.98 17.92 13.14 17.92 13.31C17.92 13.48 17.89 13.64 17.83 13.79C17.77 13.94 17.68 14.08 17.57 14.2C17.46 14.32 17.32 14.42 17.17 14.49C17.02 14.56 16.86 14.6 16.69 14.61H15.77L15.45 14.9C15.32 15.02 15.16 15.1 14.99 15.14C14.82 15.18 14.64 15.17 14.48 15.12C14.32 15.07 14.17 14.98 14.06 14.86C13.95 14.74 13.88 14.59 13.86 14.43L13 9C12.93 8.55 13.04 8.09 13.3 7.71C13.56 7.33 13.95 7.06 14.4 6.95C14.85 6.84 15.33 6.9 15.75 7.12L19.1 8.8C19.5 9 19.81 9.34 20.01 9.74C20.21 10.14 20.27 10.6 20.19 11.05L19.1 16.95Z" fill="white"/>
          </svg>
        </div>
      )}

      {/* Chatbot window */}
      {isOpen && (
        <div className="chatbot-container">
          <div className="chatbot-header">
            <div className="chatbot-title">AI Assistant</div>
            <div className="chatbot-subtitle">How can I help you today?</div>
            <button className="chatbot-close" onClick={toggleChatbot}>×</button>
          </div>
          
          <div className="chatbot-messages">
            {messages.map((message) => (
              <div 
                key={message.id} 
                className={`message ${message.sender}-message`}
              >
                {message.text}
                <div className="timestamp">{formatTime(message.timestamp)}</div>
              </div>
            ))}
            
            {isTyping && (
              <div className="message bot-message typing-indicator">
                <div className="typing-dots">
                  <div className="typing-dot"></div>
                  <div className="typing-dot"></div>
                  <div className="typing-dot"></div>
                </div>
                <span>AI is typing...</span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          
          <form onSubmit={handleSendMessage} className="chatbot-input-form">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Type your message here..."
              className="chatbot-input"
              autoFocus
            />
            <button type="submit" className="chatbot-send-button" disabled={!inputValue.trim()}>
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default Chatbot;