/**
 * API client for backend communication.
 * Handles all HTTP requests to the FastAPI backend.
 */

import axios, { AxiosInstance, AxiosError } from "axios";
import {
  QueryRequest,
  QueryResponse,
  HealthResponse,
  ApiError,
} from "../types/chat";

// Get API base URL from environment variable or use default
// Using a safe approach to handle process.env in browser environments
const getAPIBaseUrl = () => {
  if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL) {
    return process.env.REACT_APP_API_URL;
  }
  // In browser environments where process might not be defined
  return "http://localhost:8000/v1";
};

const API_BASE_URL = getAPIBaseUrl();

class ApiClient {
  private client: AxiosInstance;
  private sessionId: string | null = null;

  constructor() {
    this.client = axios.create({
      baseURL: API_BASE_URL,
      timeout: 30000, // 30 seconds
      headers: {
        "Content-Type": "application/json",
      },
    });

    // Add request interceptor to include session ID for rate limiting
    this.client.interceptors.request.use((config) => {
      if (this.sessionId) {
        config.headers["X-Session-ID"] = this.sessionId;
      }
      return config;
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError<ApiError>) => {
        return Promise.reject(this.handleError(error));
      }
    );
  }

  /**
   * Set session ID for rate limiting tracking.
   */
  setSessionId(sessionId: string) {
    this.sessionId = sessionId;
  }

  /**
   * Handle API errors and map to user-friendly messages.
   */
  private handleError(error: AxiosError<ApiError>): Error {
    if (error.response) {
      // Server responded with error status
      const { status, data } = error.response;

      switch (status) {
        case 400:
          return new Error(
            data?.message || "Invalid request. Please check your input."
          );
        case 404:
          return new Error(
            data?.message ||
              "No relevant content found. Try rephrasing your question."
          );
        case 429:
          return new Error(
            "Too many requests. Please wait a moment before trying again."
          );
        case 500:
          if (data?.error === "REQUEST_TIMEOUT") {
            return new Error(
              "Request timed out. Please try again with a simpler question."
            );
          }
          return new Error(
            "An unexpected error occurred. Please try again later."
          );
        case 503:
          return new Error(
            data?.message || "Service temporarily unavailable. Please try again later."
          );
        default:
          return new Error(data?.message || "An unexpected error occurred.");
      }
    } else if (error.request) {
      // Request made but no response received
      return new Error(
        "Unable to connect to the server. Please check your internet connection."
      );
    } else {
      // Error setting up the request
      return new Error("An unexpected error occurred.");
    }
  }

  /**
   * Submit a query to the chatbot.
   */
  async query(request: QueryRequest): Promise<QueryResponse> {
    const response = await this.client.post<QueryResponse>("/query", request);
    return response.data;
  }

  /**
   * Check backend health status.
   */
  async health(): Promise<HealthResponse> {
    const response = await this.client.get<HealthResponse>("/health");
    return response.data;
  }

  /**
   * Admin endpoint: Index book content (for testing/admin only).
   */
  async indexContent(contentPath: string, forceReindex: boolean = false): Promise<any> {
    const response = await this.client.post("/admin/index", {
      content_path: contentPath,
      force_reindex: forceReindex,
    });
    return response.data;
  }
}

// Export singleton instance
export const apiClient = new ApiClient();
