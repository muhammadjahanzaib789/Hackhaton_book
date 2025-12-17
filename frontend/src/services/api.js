import axios from 'axios';

// Get API base URL from environment variable or use default
// Using window object for browser environments to avoid process is not defined error
const getAPIBaseUrl = () => {
  // Try to get from environment variable (works in React builds)
  if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL) {
    return process.env.REACT_APP_API_URL;
  }
  // If process is not available, try window object or default
  return window.REACT_APP_API_URL || 'http://localhost:8000/v1';
};

const API_BASE_URL = getAPIBaseUrl();

// Create axios instance with default config
const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: 30000, // 30 seconds timeout
  headers: {
    'Content-Type': 'application/json',
  },
});

// Request interceptor to log requests (optional, can be removed in production)
apiClient.interceptors.request.use(
  (config) => {
    console.log(`Making request to: ${config.url}`, config.data);
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// Response interceptor to handle responses and errors
apiClient.interceptors.response.use(
  (response) => {
    console.log(`Response from: ${response.config.url}`, response.data);
    return response;
  },
  (error) => {
    console.error(`Error from: ${error.config?.url}`, error.message);
    return Promise.reject(error);
  }
);

export const chatbotAPI = {
  // Health check endpoint
  checkHealth: async () => {
    try {
      const response = await apiClient.get('/agent/health');
      return response.data;
    } catch (error) {
      throw error;
    }
  },

  // Submit a query to the RAG agent
  submitQuery: async (query, mode = 'full_book', selectedText = null, maxTokens = 500, temperature = 0.7) => {
    try {
      const requestBody = {
        query,
        mode,
        max_tokens: maxTokens,
        temperature,
      };

      // Include selected text if in selected_text mode
      if (mode === 'selected_text' && selectedText) {
        requestBody.selected_text = selectedText;
      }

      const response = await apiClient.post('/agent/query', requestBody);
      return response.data;
    } catch (error) {
      // Handle different types of errors
      if (error.response) {
        // Server responded with error status
        const errorData = {
          status: error.response.status,
          message: error.response.data?.detail || 'Server error',
          data: error.response.data,
        };
        throw errorData;
      } else if (error.request) {
        // Request was made but no response received
        const errorData = {
          status: 0,
          message: 'Network error - could not reach server',
          data: null,
        };
        throw errorData;
      } else {
        // Other errors
        const errorData = {
          status: 0,
          message: error.message || 'Unknown error',
          data: null,
        };
        throw errorData;
      }
    }
  },
};

export default apiClient;