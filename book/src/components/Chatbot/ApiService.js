/**
 * API Service for frontend-backend communication
 */

class ApiService {
  constructor(baseURL = '') {
    this.baseURL = baseURL;
  }

  /**
   * Send a chat query to the backend API
   * @param {Object} queryRequest - The query request object
   * @param {string} queryRequest.query - The user's query
   * @param {string} [queryRequest.conversation_id] - Optional conversation ID
   * @param {Object} [queryRequest.context] - Optional additional context
   * @returns {Promise<Object>} The API response
   */
  async sendChatQuery(queryRequest) {
    try {
      const response = await fetch(`${this.baseURL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(queryRequest),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error sending chat query:', error);
      throw error;
    }
  }

  /**
   * Check the health of the API
   * @returns {Promise<Object>} The health check response
   */
  async checkHealth() {
    try {
      const response = await fetch(`${this.baseURL}/health`);

      if (!response.ok) {
        throw new Error(`Health check failed with status ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error checking API health:', error);
      throw error;
    }
  }

  /**
   * Get API information
   * @returns {Promise<Object>} The API information
   */
  async getApiInfo() {
    try {
      const response = await fetch(`${this.baseURL}/`);

      if (!response.ok) {
        throw new Error(`API info request failed with status ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error getting API info:', error);
      throw error;
    }
  }
}

// Create a singleton instance
const apiService = new ApiService(process.env.REACT_APP_API_BASE_URL || '');

export default apiService;