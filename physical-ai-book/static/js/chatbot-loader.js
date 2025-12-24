// Chatbot loader for Docusaurus
// This script will be loaded on all pages and initialize the chatbot

document.addEventListener('DOMContentLoaded', function() {
  // Wait a bit for the page to fully load before initializing the chatbot
  setTimeout(function() {
    // Create the chatbot HTML structure
    const chatbotHTML = `
      <div id="chatbot-fab" class="chatbot-fab">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H17.13L16.09 18.05C15.88 18.26 15.61 18.4 15.32 18.45C15.03 18.5 14.73 18.46 14.47 18.34C14.21 18.22 13.99 18.02 13.84 17.77C13.69 17.52 13.62 17.24 13.64 16.95L14.5 12C14.5 10.4087 13.8679 8.88258 12.7426 7.75736C11.6174 6.63214 10.0913 6 8.5 6C6.9087 6 5.38258 6.63214 4.25736 7.75736C3.13214 8.88258 2.5 10.4087 2.5 12C2.5 13.5913 3.13214 15.1174 4.25736 16.2426C5.38258 17.3679 6.9087 18 8.5 18H9.5C9.76522 18 10.0196 17.8946 10.2071 17.7071C10.3946 17.5196 10.5 17.2652 10.5 17C10.5 16.7348 10.3946 16.4804 10.2071 16.2929C10.0196 16.1054 9.76522 16 9.5 16H8.5C7.43913 16 6.42172 15.5786 5.67157 14.8284C4.92143 14.0783 4.5 13.0609 4.5 12C4.5 10.9391 4.92143 9.92172 5.67157 9.17157C6.42172 8.42143 7.43913 8 8.5 8C9.56087 8 10.5783 8.42143 11.3284 9.17157C12.0786 9.92172 12.5 10.9391 12.5 12C12.5 12.1904 12.4846 12.379 12.4548 12.5632C12.425 12.7475 12.3812 12.9262 12.3244 13.0972C12.2676 13.2682 12.1983 13.4303 12.1177 13.5817C12.0371 13.7331 11.9459 13.8728 11.8459 14H15.27L17.95 16.68C18.06 16.79 18.2 16.86 18.35 16.88C18.5 16.9 18.65 16.87 18.78 16.79C18.91 16.71 19.01 16.59 19.07 16.45C19.13 16.31 19.14 16.15 19.1 16L20 11.05C20.02 10.76 19.95 10.48 19.79 10.23C19.63 9.98 19.41 9.78 19.15 9.66C18.89 9.54 18.6 9.5 18.31 9.57C18.02 9.64 17.76 9.79 17.57 10L16.5 11C16.28 11.2 16.01 11.35 15.71 11.42C15.41 11.49 15.1 11.48 14.81 11.39L13.95 9.05C13.85 8.78 13.82 8.49 13.86 8.2C13.9 7.91 14.02 7.64 14.2 7.42C14.38 7.2 14.61 7.03 14.88 6.93C15.15 6.83 15.44 6.81 15.72 6.87L17.88 7.29C18.1 7.33 18.3 7.42 18.47 7.55C18.64 7.68 18.77 7.85 18.85 8.04C18.93 8.23 18.96 8.44 18.94 8.65C18.92 8.86 18.85 9.06 18.74 9.23L15.77 12.16C15.84 12.13 15.91 12.11 15.99 12.09C16.07 12.07 16.14 12.05 16.22 12.04C16.3 12.03 16.37 12.02 16.45 12.01C16.53 12 16.61 12 16.69 12.01C16.86 12.02 17.02 12.06 17.17 12.13C17.32 12.2 17.46 12.3 17.57 12.42C17.68 12.54 17.77 12.68 17.83 12.83C17.89 12.98 17.92 13.14 17.92 13.31C17.92 13.48 17.89 13.64 17.83 13.79C17.77 13.94 17.68 14.08 17.57 14.2C17.46 14.32 17.32 14.42 17.17 14.49C17.02 14.56 16.86 14.6 16.69 14.61H15.77L15.45 14.9C15.32 15.02 15.16 15.1 14.99 15.14C14.82 15.18 14.64 15.17 14.48 15.12C14.32 15.07 14.17 14.98 14.06 14.86C13.95 14.74 13.88 14.59 13.86 14.43L13 9C12.93 8.55 13.04 8.09 13.3 7.71C13.56 7.33 13.95 7.06 14.4 6.95C14.85 6.84 15.33 6.9 15.75 7.12L19.1 8.8C19.5 9 19.81 9.34 20.01 9.74C20.21 10.14 20.27 10.6 20.19 11.05L19.1 16.95Z" fill="white"/>
        </svg>
      </div>

      <div id="chatbot-container" class="chatbot-container" style="display: none;">
        <div class="chatbot-header">
          <div class="chatbot-title">Physical AI Assistant</div>
          <div class="chatbot-subtitle">Ask about robotics, ROS 2, or VLA</div>
          <button id="chatbot-close" class="chatbot-close">×</button>
        </div>

        <div class="chatbot-messages" id="chatMessages">
          <div class="message bot-message">
            Hello! 👋 I'm your Physical AI assistant. How can I help you today?
            <div class="timestamp" id="welcomeTime"></div>
          </div>
        </div>

        <div class="typing-indicator" id="typingIndicator">
          <div class="typing-dots">
            <div class="typing-dot"></div>
            <div class="typing-dot"></div>
            <div class="typing-dot"></div>
          </div>
          <span>AI is typing...</span>
        </div>

        <form id="chatForm" class="chatbot-input-form">
          <input
            type="text"
            id="userInput"
            class="chatbot-input"
            placeholder="Ask about Physical AI, ROS 2, or robotics..."
            autocomplete="off"
          >
          <button type="submit" id="sendButton" class="chatbot-send-button">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          </button>
        </form>
      </div>
    `;

    // Append chatbot HTML to the body
    document.body.insertAdjacentHTML('beforeend', chatbotHTML);

    // Add chatbot CSS
    const chatbotCSS = `
      .chatbot-fab {
        position: fixed;
        bottom: 20px;
        right: 20px;
        width: 60px;
        height: 60px;
        background: linear-gradient(135deg, #667eea, #764ba2);
        border-radius: 50%;
        display: flex;
        align-items: center;
        justify-content: center;
        cursor: pointer;
        box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
        z-index: 10000;
        transition: all 0.3s ease;
        border: none;
        font-size: 0;
      }

      .chatbot-fab:hover {
        transform: scale(1.1);
        box-shadow: 0 6px 25px rgba(0, 0, 0, 0.4);
      }

      .chatbot-fab svg {
        transition: transform 0.3s ease;
        cursor: pointer;
      }

      .chatbot-fab:hover svg {
        transform: scale(1.2);
      }

      .chatbot-container {
        position: fixed;
        bottom: 20px;
        right: 20px;
        width: 380px;
        height: 500px;
        background: white;
        border-radius: 12px;
        box-shadow: 0 10px 40px rgba(0, 0, 0, 0.2);
        display: flex;
        flex-direction: column;
        overflow: hidden;
        z-index: 10000;
        animation: slideUp 0.3s ease-out;
      }

      @keyframes slideUp {
        from {
          transform: translateY(100%);
          opacity: 0;
        }
        to {
          transform: translateY(0);
          opacity: 1;
        }
      }

      .chatbot-header {
        background: linear-gradient(135deg, #667eea, #764ba2);
        color: white;
        padding: 18px 20px;
        position: relative;
        display: flex;
        align-items: center;
        justify-content: space-between;
      }

      .chatbot-title {
        font-size: 1.2em;
        font-weight: 600;
      }

      .chatbot-subtitle {
        font-size: 0.85em;
        opacity: 0.9;
        margin-top: 3px;
      }

      .chatbot-close {
        background: none;
        border: none;
        color: white;
        font-size: 24px;
        cursor: pointer;
        width: 30px;
        height: 30px;
        display: flex;
        align-items: center;
        justify-content: center;
        border-radius: 50%;
        transition: background-color 0.2s;
      }

      .chatbot-close:hover {
        background-color: rgba(255, 255, 255, 0.2);
      }

      .chatbot-messages {
        flex: 1;
        padding: 20px;
        overflow-y: auto;
        display: flex;
        flex-direction: column;
        gap: 12px;
        background: #fafafa;
      }

      .message {
        max-width: 80%;
        padding: 12px 16px;
        border-radius: 18px;
        font-size: 14px;
        line-height: 1.4;
        position: relative;
        animation: fadeIn 0.3s ease-out;
      }

      @keyframes fadeIn {
        from {
          opacity: 0;
          transform: translateY(10px);
        }
        to {
          opacity: 1;
          transform: translateY(0);
        }
      }

      .user-message {
        background: #007bff;
        color: white;
        align-self: flex-end;
        border-bottom-right-radius: 5px;
      }

      .bot-message {
        background: #e9ecef;
        color: #333;
        align-self: flex-start;
        border-bottom-left-radius: 5px;
      }

      .timestamp {
        font-size: 0.7em;
        opacity: 0.7;
        margin-top: 5px;
        text-align: right;
      }

      .typing-indicator {
        display: none;
        background: #e9ecef;
        color: #666;
        padding: 12px 16px;
        border-radius: 18px;
        align-self: flex-start;
        font-size: 14px;
        margin-top: 10px;
      }

      .typing-indicator.active {
        display: flex;
        align-items: center;
        gap: 8px;
      }

      .typing-dots {
        display: flex;
        gap: 3px;
      }

      .typing-dot {
        width: 8px;
        height: 8px;
        background: #666;
        border-radius: 50%;
        animation: bounce 1.5s infinite;
      }

      .typing-dot:nth-child(2) {
        animation-delay: 0.2s;
      }

      .typing-dot:nth-child(3) {
        animation-delay: 0.4s;
      }

      @keyframes bounce {
        0%, 100% {
          transform: translateY(0);
        }
        50% {
          transform: translateY(-5px);
        }
      }

      .chatbot-input-form {
        padding: 15px;
        background: white;
        border-top: 1px solid #eee;
        display: flex;
        gap: 10px;
      }

      .chatbot-input {
        flex: 1;
        padding: 12px 15px;
        border: 2px solid #e9ecef;
        border-radius: 25px;
        outline: none;
        font-size: 14px;
        transition: border-color 0.3s;
      }

      .chatbot-input:focus {
        border-color: #007bff;
      }

      .chatbot-send-button {
        background: #007bff;
        color: white;
        border: none;
        border-radius: 50%;
        width: 45px;
        height: 45px;
        cursor: pointer;
        display: flex;
        align-items: center;
        justify-content: center;
        transition: background-color 0.3s;
      }

      .chatbot-send-button:hover:not(:disabled) {
        background: #0056b3;
      }

      .chatbot-send-button:disabled {
        background: #ccc;
        cursor: not-allowed;
      }

      /* Responsive design */
      @media (max-width: 480px) {
        .chatbot-container {
          width: calc(100vw - 40px);
          height: 70vh;
          bottom: 10px;
          right: 10px;
        }

        .message {
          max-width: 90%;
        }

        .chatbot-fab {
          width: 55px;
          height: 55px;
          bottom: 15px;
          right: 15px;
        }
      }

      /* Scrollbar styling */
      .chatbot-messages::-webkit-scrollbar {
        width: 6px;
      }

      .chatbot-messages::-webkit-scrollbar-track {
        background: #f1f1f1;
      }

      .chatbot-messages::-webkit-scrollbar-thumb {
        background: #c1c1c1;
        border-radius: 3px;
      }

      .chatbot-messages::-webkit-scrollbar-thumb:hover {
        background: #a8a8a8;
      }
    `;

    // Create and append style element
    const styleElement = document.createElement('style');
    styleElement.textContent = chatbotCSS;
    document.head.appendChild(styleElement);

    // Get DOM elements
    const chatbotFab = document.getElementById('chatbot-fab');
    const chatbotContainer = document.getElementById('chatbot-container');
    const chatCloseBtn = document.getElementById('chatbot-close');
    const chatMessages = document.getElementById('chatMessages');
    const userInput = document.getElementById('userInput');
    const chatForm = document.getElementById('chatForm');
    const typingIndicator = document.getElementById('typingIndicator');

    // Set welcome message timestamp
    document.getElementById('welcomeTime').textContent = getCurrentTime();

    // Sample responses for the chatbot
    const responses = {
      greetings: [
        "Hello! How can I assist you with Physical AI today?",
        "Hi there! What can I do for you regarding robotics?",
        "Greetings! How may I help you with the Physical AI content?"
      ],
      thanks: [
        "You're welcome!",
        "Happy to help!",
        "Anytime! Is there anything else I can assist with?"
      ],
      help: [
        "I can help you with general inquiries about the Physical AI & Humanoid Robotics content.",
        "I'm here to answer your questions about the modules and lessons.",
        "Feel free to ask me anything about ROS 2, Digital Twins, NVIDIA Isaac, or VLA Pipeline!"
      ],
      goodbye: [
        "Goodbye! Feel free to come back if you have more questions about Physical AI.",
        "See you later! Have a great day exploring robotics!",
        "Take care! I'm always here if you need assistance with the content."
      ],
      default: [
        "I'm not sure I understand. Could you rephrase that?",
        "Can you provide more details about your question regarding the content?",
        "I'm still learning. Could you try asking in a different way?",
        "Interesting question! Unfortunately, I don't have enough information to provide a detailed answer right now."
      ]
    };

    // Common keywords and phrases to detect user intent
    const keywordPatterns = [
      { keywords: ['hello', 'hi', 'hey', 'greetings'], responses: responses.greetings },
      { keywords: ['thank', 'thanks', 'thank you', 'appreciate'], responses: responses.thanks },
      { keywords: ['help', 'assist', 'support', 'guidance'], responses: responses.help },
      { keywords: ['bye', 'goodbye', 'see you', 'farewell'], responses: responses.goodbye },
      { keywords: ['ros', 'ros2', 'nervous system'], responses: ["ROS 2 is the nervous system for robots. You can learn more in Module 1 of our content."] },
      { keywords: ['digital twin', 'gazebo', 'simulation'], responses: ["Digital twins are covered in Module 2. This allows you to create virtual replicas for testing and development."] },
      { keywords: ['nvidia', 'isaac', 'simulation'], responses: ["NVIDIA Isaac is covered in Module 3. It provides tools for robotics simulation and development."] },
      { keywords: ['vla', 'voice', 'pipeline'], responses: ["The VLA Pipeline is covered in Module 4. It deals with voice and language processing for robotics."] }
    ];

    // Function to get current time in HH:MM format
    function getCurrentTime() {
      const now = new Date();
      return now.getHours().toString().padStart(2, '0') + ':' +
             now.getMinutes().toString().padStart(2, '0');
    }

    // Function to add a message to the chat
    function addMessage(text, isUser = false) {
      const messageDiv = document.createElement('div');
      messageDiv.className = `message ${isUser ? 'user-message' : 'bot-message'}`;

      messageDiv.innerHTML = `${text}<div class="timestamp">${getCurrentTime()}</div>`;
      chatMessages.appendChild(messageDiv);

      // Scroll to bottom
      chatMessages.scrollTop = chatMessages.scrollHeight;
    }

    // Function to simulate bot thinking and respond
    function respondToUser(userMessage) {
      // Show typing indicator
      typingIndicator.classList.add('active');

      // Simulate thinking time (random between 1-2 seconds)
      setTimeout(() => {
        // Hide typing indicator
        typingIndicator.classList.remove('active');

        // Convert message to lowercase for easier matching
        const lowerMsg = userMessage.toLowerCase();

        // Check for matches in our patterns
        let foundResponse = false;

        for (const pattern of keywordPatterns) {
          if (pattern.keywords.some(keyword => lowerMsg.includes(keyword))) {
            const randomResponse = pattern.responses[Math.floor(Math.random() * pattern.responses.length)];
            addMessage(randomResponse, false);
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
              "That's an interesting question about Physical AI. Based on what I know, ",
              "I'd be happy to address that about robotics. ",
              "Let me think about that for a moment regarding your robotics question. ",
              "Great question about robotics! "
            ];
            const followUps = [
              "I recommend checking our documentation in the relevant modules for more details.",
              "Could you provide more context so I can better assist you with the content?",
              "I can help you find the information you need in our modules.",
              "I suggest exploring our modules on ROS 2, Digital Twins, NVIDIA Isaac, or VLA Pipeline for related information."
            ];

            const randomQuestionResponse = questionResponses[Math.floor(Math.random() * questionResponses.length)];
            const randomFollowUp = followUps[Math.floor(Math.random() * followUps.length)];
            addMessage(randomQuestionResponse + randomFollowUp, false);
          } else {
            // Use default responses for general messages
            const randomResponse = responses.default[Math.floor(Math.random() * responses.default.length)];
            addMessage(randomResponse, false);
          }
        }
      }, 1000 + Math.random() * 1000); // Random delay between 1-2 seconds
    }

    // Function to handle sending a message
    function sendMessage(e) {
      e.preventDefault();

      const message = userInput.value.trim();

      if (message) {
        // Add user's message to chat
        addMessage(message, true);

        // Clear input field
        userInput.value = '';

        // Respond to user
        respondToUser(message);
      }
    }

    // Function to toggle chatbot open/close
    function toggleChatbot() {
      if (chatbotContainer.style.display === 'none' || !chatbotContainer.style.display) {
        chatbotContainer.style.display = 'flex';
        chatbotFab.style.display = 'none';
      } else {
        chatbotContainer.style.display = 'none';
        chatbotFab.style.display = 'flex';
      }
    }

    // Event listeners
    chatbotFab.addEventListener('click', toggleChatbot);
    chatCloseBtn.addEventListener('click', toggleChatbot);
    chatForm.addEventListener('submit', sendMessage);

    userInput.addEventListener('keypress', function(e) {
      if (e.key === 'Enter') {
        sendMessage(e);
      }
    });

    // Add initial sample message
    setTimeout(() => {
      addMessage("This chatbot is now integrated into your Physical AI website! Try asking me about robotics, ROS 2, or VLA.", false);
    }, 2000);
  }, 1000); // Delay to ensure page is loaded
});