import React, { useState, useEffect, useRef } from 'react';
import './ChatInterface.css';

const ChatInterface = () => {
  const [messages, setMessages] = useState(() => {
    // Load conversation history from sessionStorage
    const savedMessages = sessionStorage.getItem('chatMessages');
    return savedMessages ? JSON.parse(savedMessages) : [];
  });
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);

  // Save messages to sessionStorage whenever messages change
  useEffect(() => {
    sessionStorage.setItem('chatMessages', JSON.stringify(messages));
  }, [messages]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Call the backend API directly (CORS is enabled on the backend)
      const response = await fetch('http://127.0.0.1:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response to chat
      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: data.response,
        sources: data.sources,
        timestamp: data.timestamp,
        retrievalResults: data.retrieval_results
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      setError(err.message);
      console.error('Error sending message:', err);

      // Add error message to chat
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error while processing your request. Please try again.',
        isError: true,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <h3>AI Assistant</h3>
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your AI assistant. How can I help you today?</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.role} ${message.isError ? 'error' : ''}`}
            >
              <div className="message-content">
                <div className="message-text">{message.content}</div>

              </div>
            </div>
          ))
        )}

        {isLoading && (
          <div className="message assistant">
            <div className="message-content">
              <div className="loading-indicator">Thinking...</div>
            </div>
          </div>
        )}

        {error && (
          <div className="message assistant error">
            <div className="message-content">
              <div className="error-message">Error: {error}</div>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className="chat-input-form">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Type your question here..."
          disabled={isLoading}
          className="chat-input"
        />
        <button type="submit" disabled={isLoading || !inputValue.trim()} className="send-button">
          Send
        </button>
      </form>
    </div>
  );
};

export default ChatInterface;