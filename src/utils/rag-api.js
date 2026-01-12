/**
 * API client for RAG chat functionality
 */

const API_BASE_URL = process.env.REACT_APP_API_URL || '/api/v1';

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

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
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

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
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
  }
};