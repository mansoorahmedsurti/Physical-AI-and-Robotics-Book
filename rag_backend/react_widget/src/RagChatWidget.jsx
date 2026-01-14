import React, { useState, useEffect, useRef } from 'react';
import './RagChatWidget.css';

const RagChatWidget = ({ backendUrl = 'https://mansoorahmedsurti-rag-chatbot-robotics.hf.space', documentContext = 'general' }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversationId, setConversationId] = useState(null);
  const messagesEndRef = useRef(null);

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to send message to backend
  const sendMessage = async () => {
    if (!inputMessage.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputMessage,
      timestamp: new Date().toISOString()
    };

    // Add user message to UI immediately
    setMessages(prev => [...prev, userMessage]);
    setInputMessage('');
    setIsLoading(true);

    try {
      // Prepare request payload
      const requestBody = {
        message: inputMessage,
        temperature: 0.7
      };

      // Add conversation ID if we have one
      if (conversationId) {
        requestBody.conversation_id = conversationId;
      }

      // Send message to backend
      const response = await fetch(`${backendUrl}/api/v1/chat/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Update conversation ID if it's new
      if (!conversationId && data.conversation_id) {
        setConversationId(data.conversation_id);
      }

      // Create assistant message
      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: data.response,
        sources: data.sources || [],
        timestamp: new Date().toISOString()
      };

      // Add assistant message to UI
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to UI
      const errorMessage = {
        id: Date.now() + 1,
        role: 'error',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle form submission
  const handleSubmit = (e) => {
    e.preventDefault();
    sendMessage();
  };

  // Toggle chat widget open/close
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Start a new conversation
  const startNewConversation = async () => {
    try {
      const response = await fetch(`${backendUrl}/api/v1/chat/conversation/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({})
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      setConversationId(data.conversation_id);
      setMessages([]);
    } catch (error) {
      console.error('Error starting new conversation:', error);
    }
  };

  return (
    <div className="rag-chat-widget">
      {isOpen ? (
        <div className="chat-container">
          <div className="chat-header">
            <div className="chat-title">Book Assistant</div>
            <div className="chat-controls">
              <button onClick={startNewConversation} className="new-conversation-btn" title="New conversation">
                ✚
              </button>
              <button onClick={toggleChat} className="close-btn">✕</button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your book assistant. Ask me anything about the book content.</p>
              </div>
            ) : (
              messages.map((msg) => (
                <div key={msg.id} className={`message ${msg.role}`}>
                  <div className="message-content">
                    {msg.content}
                  </div>
                  {msg.sources && msg.sources.length > 0 && (
                    <div className="message-sources">
                      <small>Sources:</small>
                      <ul>
                        {msg.sources.slice(0, 3).map((source, idx) => (
                          <li key={idx}>
                            Doc: {source.content_preview.substring(0, 50)}...
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className="message assistant">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className="chat-input-form">
            <input
              type="text"
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              placeholder="Ask about the book..."
              disabled={isLoading}
              className="chat-input"
            />
            <button type="submit" disabled={isLoading || !inputMessage.trim()} className="send-button">
              Send
            </button>
          </form>
        </div>
      ) : (
        <button onClick={toggleChat} className="chat-toggle-button">
          💬 Ask Book Assistant
        </button>
      )}
    </div>
  );
};

export default RagChatWidget;