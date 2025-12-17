import { chatbotAPI } from './api';

export const chatbotService = {
  // Submit a query to the backend RAG service
  submitQuery: async (query, mode = 'full_book', selectedText = null) => {
    try {
      const response = await chatbotAPI.submitQuery(query, mode, selectedText);
      return {
        success: true,
        data: response,
      };
    } catch (error) {
      console.error('Error submitting query:', error);
      return {
        success: false,
        error: {
          message: error.message || 'Unknown error occurred',
          status: error.status || 0,
          details: error.details || null,
        },
      };
    }
  },

  // Check if the backend service is healthy
  checkHealth: async () => {
    try {
      const response = await chatbotAPI.checkHealth();
      return {
        success: true,
        data: response,
      };
    } catch (error) {
      console.error('Health check failed:', error);
      return {
        success: false,
        error: {
          message: error.message || 'Health check failed',
          status: error.status || 0,
        },
      };
    }
  },

  // Process the response from the backend
  processResponse: (responseData) => {
    // Validate the response structure
    if (!responseData || !responseData.answer) {
      throw new Error('Invalid response format from backend');
    }

    // Return the processed response
    return {
      answer: responseData.answer,
      citations: responseData.citations || [],
      query: responseData.query || '',
      tokensUsed: responseData.tokens_used || 0,
      confidence: responseData.confidence || null,
    };
  },

  // Implement retry logic for failed API requests
  submitQueryWithRetry: async (query, mode = 'full_book', selectedText = null, maxRetries = 3) => {
    let lastError;

    for (let i = 0; i < maxRetries; i++) {
      try {
        const result = await chatbotService.submitQuery(query, mode, selectedText);
        if (result.success) {
          return result;
        }
        lastError = result.error;

        // Wait before retrying (exponential backoff)
        await new Promise(resolve => setTimeout(resolve, Math.pow(2, i) * 1000));
      } catch (error) {
        lastError = error;
        // Wait before retrying (exponential backoff)
        await new Promise(resolve => setTimeout(resolve, Math.pow(2, i) * 1000));
      }
    }

    // If all retries failed, return the last error
    return {
      success: false,
      error: lastError
    };
  },

  // Add timeout handling for API requests (already implemented in the API client)
  // This is more of a placeholder since timeout is handled at the API client level
  submitQueryWithTimeout: async (query, mode = 'full_book', selectedText = null, timeoutMs = 30000) => {
    // Currently using timeout from axios config, but we could implement custom timeout handling here
    return await chatbotService.submitQuery(query, mode, selectedText);
  }
};