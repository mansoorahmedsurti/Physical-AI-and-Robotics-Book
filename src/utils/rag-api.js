/**
 * API client for RAG chat functionality
 */

// Determine the API base URL based on environment
// For Vercel deployments, use environment variables to configure the backend URL
const getApiBaseUrl = () => {
  // Check for environment variables first (these will be available in browser)
  const envUrl = typeof process !== 'undefined' ? (process.env.REACT_APP_API_URL || process.env.NEXT_PUBLIC_API_URL) : undefined;
  if (envUrl) {
    return envUrl.endsWith('/api/v1') ? envUrl : `${envUrl}/api/v1`;
  }

  // Use the deployed backend URL for production
  const prodUrl = 'https://mansoorahmedsurti-rag-chatbot-robotics.hf.space/api/v1';
  console.log('Using production RAG API URL:', prodUrl);
  return prodUrl;
};

const API_BASE_URL = getApiBaseUrl();

// Check if we're in a browser environment
const isBrowser = typeof window !== 'undefined';

// Log the API URL being used for debugging
if (isBrowser && typeof window.console !== 'undefined') {
  console.log('RAG API Base URL:', API_BASE_URL);
}

/**
 * Object containing all RAG API functions
 */
export const ragApi = {
  /**
   * Send a chat message and get a response
   * @param {string} message - The user's message
   * @param {string|null} conversationId - Optional conversation ID for continuing a conversation
   * @param {number} temperature - Temperature parameter for response variability
   * @returns {Promise<Object>} Response containing conversation_id, response, and sources
   */
  async chat(message, conversationId = null, temperature = 0.7) {
    try {
      console.log('Making chat API call to:', `${API_BASE_URL}/chat/`);
      console.log('With payload:', { message, conversation_id: conversationId, temperature });

      const response = await fetch(`${API_BASE_URL}/chat/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Add authorization header if needed
          ...(typeof window !== 'undefined' && localStorage.getItem('auth_token') && {
            'Authorization': `Bearer ${localStorage.getItem('auth_token')}`
          })
        },
        body: JSON.stringify({
          message: message,
          conversation_id: conversationId,
          temperature: temperature
        })
      });

      console.log('Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('API error response:', errorText);
        throw new Error(`HTTP error! status: ${response.status}, message: ${errorText}`);
      }

      const data = await response.json();
      console.log('API response data:', data);
      return data;
    } catch (error) {
      console.error('Error in chat API call:', error);
      throw error;
    }
  },

  /**
   * Create a new conversation
   * @param {string} title - Optional title for the conversation
   * @returns {Promise<Object>} Response containing conversation_id
   */
  async createConversation(title = null) {
    try {
      console.log('Making create conversation API call to:', `${API_BASE_URL}/chat/conversation/`);
      console.log('With payload:', { title });

      const response = await fetch(`${API_BASE_URL}/chat/conversation/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Add authorization header if needed
          ...(typeof window !== 'undefined' && localStorage.getItem('auth_token') && {
            'Authorization': `Bearer ${localStorage.getItem('auth_token')}`
          })
        },
        body: JSON.stringify({
          title: title
        })
      });

      console.log('Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('API error response:', errorText);
        throw new Error(`HTTP error! status: ${response.status}, message: ${errorText}`);
      }

      const data = await response.json();
      console.log('API response data:', data);
      return data;
    } catch (error) {
      console.error('Error in create conversation API call:', error);
      throw error;
    }
  },

  /**
   * Get conversation history
   * @param {string} conversationId - The conversation ID to retrieve
   * @returns {Promise<Object>} Response containing conversation history
   */
  async getConversationHistory(conversationId) {
    try {
      const response = await fetch(`${API_BASE_URL}/chat/conversation/${conversationId}`, {
        headers: {
          // Add authorization header if needed
          ...(typeof window !== 'undefined' && localStorage.getItem('auth_token') && {
            'Authorization': `Bearer ${localStorage.getItem('auth_token')}`
          })
        }
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error in get conversation history API call:', error);
      throw error;
    }
  },

  /**
   * Upload a document for RAG processing
   * @param {File} file - The file to upload
   * @returns {Promise<Object>} Response containing document details
   */
  async uploadDocument(file) {
    try {
      const formData = new FormData();
      formData.append('file', file);

      const response = await fetch(`${API_BASE_URL}/documents/ingest`, {
        method: 'POST',
        // Don't set Content-Type header when using FormData (browser sets it with boundary)
        headers: {
          // Add authorization header if needed
          ...(typeof window !== 'undefined' && localStorage.getItem('auth_token') && {
            'Authorization': `Bearer ${localStorage.getItem('auth_token')}`
          })
        },
        body: formData
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error in upload document API call:', error);
      throw error;
    }
  },

  /**
   * Health check to see if the backend is reachable
   * @returns {Promise<boolean>} Whether the backend is accessible
   */
  async healthCheck() {
    try {
      // Try to reach the health endpoint first, fall back to a simple GET
      const healthUrl = `${API_BASE_URL.replace('/api/v1', '')}/health`;
      const response = await fetch(healthUrl, {
        method: 'GET',
        headers: {
          'Accept': 'application/json',
        }
      });

      return response.ok;
    } catch (error) {
      console.warn('Health check failed:', error);
      return false;
    }
  }
};