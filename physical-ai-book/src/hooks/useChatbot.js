import { useState, useCallback, useEffect } from 'react';
import { chatbotService } from '@site/src/services/chatbotService';

const useChatbot = () => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [currentMode, setCurrentMode] = useState('full_book');
  const [isClient, setIsClient] = useState(false);
  const [selectedText, setSelectedText] = useState('');

  // Initialize client-side only features after component mounts
  useEffect(() => {
    setIsClient(true);

    // Try to get mode from localStorage to persist across page navigations
    if (typeof window !== 'undefined' && window.localStorage) {
      const savedMode = localStorage.getItem('chatbot-mode');
      if (savedMode) {
        setCurrentMode(savedMode);
      }
    }
  }, []);

  // Save current mode to localStorage when it changes (client-side only)
  useEffect(() => {
    if (isClient && typeof window !== 'undefined' && window.localStorage) {
      localStorage.setItem('chatbot-mode', currentMode);
    }
  }, [currentMode, isClient]);

  // Add a message to the conversation
  const addMessage = useCallback((message) => {
    setMessages(prev => [...prev, message]);
  }, []);

  // Clear the conversation
  const clearConversation = useCallback(() => {
    setMessages([]);
  }, []);

  // Get currently selected text from the page
  const getSelectedText = useCallback(() => {
    if (window.getSelection) {
      return window.getSelection().toString().trim();
    } else if (document.selection && document.selection.type !== 'Control') {
      // For older IE versions
      return document.selection.createRange().text.trim();
    }
    return '';
  }, []);

  // Handle sending a message to the backend
  const sendMessage = useCallback(async (query, mode = 'full_book', selectedTextValue = null) => {
    if (!query.trim()) return;

    // Update current mode if it has changed
    if (mode !== currentMode) {
      setCurrentMode(mode);
    }

    // Get selected text if in selected_text mode and no value was passed
    let textToUse = selectedTextValue;
    if (mode === 'selected_text' && !textToUse) {
      textToUse = getSelectedText();
      if (!textToUse) {
        // Show an alert or notification to the user about selecting text
        alert('Please select text on the page before using selected text mode.');
        return;
      }
    }

    // Update selected text if provided
    if (textToUse !== null) {
      setSelectedText(textToUse);
    }

    // Add user message to the conversation
    const userMessage = {
      id: Date.now(),
      text: query,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Submit query to the backend with retry logic
      const result = await chatbotService.submitQueryWithRetry(query, mode, textToUse);

      if (result.success) {
        // Process the response
        const processedResponse = chatbotService.processResponse(result.data);

        // Create bot message
        const botMessage = {
          id: Date.now() + 1,
          text: processedResponse.answer,
          sender: 'bot',
          timestamp: new Date(),
          citations: processedResponse.citations,
        };

        setMessages(prev => [...prev, botMessage]);
      } else {
        // Handle error response
        const errorMessage = {
          id: Date.now() + 1,
          text: `Error: ${result.error.message}`,
          sender: 'bot',
          timestamp: new Date(),
          isError: true,
        };

        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      // Handle unexpected errors
      const errorMessage = {
        id: Date.now() + 1,
        text: `Unexpected error: ${error.message}`,
        sender: 'bot',
        timestamp: new Date(),
        isError: true,
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  }, [currentMode, getSelectedText]);

  // Check if the backend service is healthy
  const checkHealth = useCallback(async () => {
    try {
      const result = await chatbotService.checkHealth();
      return result;
    } catch (error) {
      console.error('Health check error:', error);
      return { success: false, error: { message: error.message } };
    }
  }, []);

  // Toggle between full book and selected text mode
  const toggleMode = useCallback((mode) => {
    setCurrentMode(mode);
  }, []);

  // Update UI to reflect the current mode (already handled by the state)
  const updateUIForCurrentMode = useCallback(() => {
    // This is handled automatically by React's reactivity
    // But we could add additional UI updates here if needed
  }, []);

  // Implement mode persistence across page navigation
  const persistModeAcrossNavigation = useCallback(() => {
    // Mode is already persisted in localStorage via the useEffect hook
  }, []);

  return {
    messages,
    isLoading,
    currentMode,
    selectedText,
    addMessage,
    clearConversation,
    sendMessage,
    checkHealth,
    toggleMode,
    getSelectedText,
    updateUIForCurrentMode,
    persistModeAcrossNavigation,
  };
};

export default useChatbot;