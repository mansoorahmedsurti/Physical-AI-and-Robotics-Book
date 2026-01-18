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
    // Retry logic for Hugging Face Spaces that may be sleeping
    const maxRetries = 3;
    const baseDelay = 2000; // 2 seconds base delay

    for (let attempt = 1; attempt <= maxRetries; attempt++) {
      try {
        console.log(`Making chat API call to: ${API_BASE_URL}/chat/public/ (attempt ${attempt})`);
        console.log('With payload:', { message, conversation_id: conversationId, temperature });

        // Set a longer timeout for the fetch request to accommodate Hugging Face Space wake-up time
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 second timeout

        const response = await fetch(`${API_BASE_URL}/chat/public/`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            // Remove authorization header for public endpoint
          },
          body: JSON.stringify({
            message: message,
            conversation_id: conversationId,
            temperature: temperature
          }),
          signal: controller.signal
        });

        clearTimeout(timeoutId);

        console.log('Response status:', response.status);

        if (!response.ok) {
          const errorText = await response.text().catch(() => 'Unable to read error response');
          console.error('API error response:', errorText);

          // If it's a 503 (Service Unavailable) or 504 (Gateway Timeout), likely the space is sleeping
          if (response.status === 503 || response.status === 504 || response.status === 502) {
            if (attempt < maxRetries) {
              console.log(`Backend may be sleeping, retrying in ${attempt * baseDelay}ms...`);
              await new Promise(resolve => setTimeout(resolve, attempt * baseDelay));
              continue; // Retry
            }
          }

          throw new Error(`HTTP error! status: ${response.status}, message: ${errorText}`);
        }

        const data = await response.json();
        console.log('API response data:', data);
        return data;
      } catch (error) {
        console.error(`Error in chat API call (attempt ${attempt}):`, error);

        // If it's an abort error (timeout), and we have more retries, continue
        if (error.name === 'AbortError' && attempt < maxRetries) {
          console.log(`Request timed out, retrying in ${attempt * baseDelay}ms...`);
          await new Promise(resolve => setTimeout(resolve, attempt * baseDelay));
          continue; // Retry
        }

        // If we've exhausted retries, throw the error
        if (attempt === maxRetries) {
          throw error;
        }
      }
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
    // Retry logic for Hugging Face Spaces that may be sleeping
    const maxRetries = 2;
    const baseDelay = 1000; // 1 second base delay

    for (let attempt = 1; attempt <= maxRetries; attempt++) {
      try {
        // Try to reach the health endpoint first, fall back to a simple GET
        const healthUrl = `${API_BASE_URL.replace('/api/v1', '')}/health`;

        console.log(`Making health check to: ${healthUrl} (attempt ${attempt})`);

        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 15000); // 15 second timeout

        const response = await fetch(healthUrl, {
          method: 'GET',
          headers: {
            'Accept': 'application/json',
          },
          signal: controller.signal
        });

        clearTimeout(timeoutId);

        const isHealthy = response.ok;
        console.log(`Health check result: ${isHealthy ? 'healthy' : 'unhealthy'} (status: ${response.status})`);

        return isHealthy;
      } catch (error) {
        console.warn(`Health check failed (attempt ${attempt}):`, error);

        if (attempt < maxRetries) {
          console.log(`Backend may be sleeping, retrying in ${attempt * baseDelay}ms...`);
          await new Promise(resolve => setTimeout(resolve, attempt * baseDelay));
          continue; // Retry
        }

        return false;
      }
    }
  }
};