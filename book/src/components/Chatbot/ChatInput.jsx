import React, { useState } from 'react';

const ChatInput = ({ onSendMessage, disabled = false }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!inputValue.trim() || disabled) return;

    onSendMessage(inputValue);
    setInputValue('');
  };

  return (
    <form onSubmit={handleSubmit} className="chat-input-form">
      <input
        type="text"
        value={inputValue}
        onChange={(e) => setInputValue(e.target.value)}
        placeholder="Type your question here..."
        disabled={disabled}
        className="chat-input"
      />
      <button
        type="submit"
        disabled={disabled || !inputValue.trim()}
        className="send-button"
      >
        Send
      </button>
    </form>
  );
};

export default ChatInput;