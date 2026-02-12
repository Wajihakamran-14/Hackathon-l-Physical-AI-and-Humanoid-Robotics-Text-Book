import React, { useState } from 'react';
import ChatInterface from './ChatInterface';
import './FloatingChatButton.css';

const FloatingChatButton = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    // Clear sessionStorage when closing the chat
    if (isOpen) {
      sessionStorage.removeItem('chatMessages');
    }
  };

  return (
    <div className="floating-chat-container">
      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <h3>AI Assistant</h3>
            <button onClick={toggleChat} className="close-button">✕</button>
          </div>
          <div className="chat-body">
            <ChatInterface />
          </div>
        </div>
      )}

      <button
        className={`floating-chat-button ${isOpen ? 'hidden' : ''}`}
        onClick={toggleChat}
        aria-label="Open chat"
      >
        💬
      </button>
    </div>
  );
};

export default FloatingChatButton;