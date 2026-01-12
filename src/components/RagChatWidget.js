import React, { useState, useEffect, useRef } from 'react';
import { ragApi } from '../utils/rag-api';
import '../css/rag-chat.css';

const RagChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your RAG assistant. Ask me questions about the robotics book content.", sender: 'bot' }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversations, setConversations] = useState([]);
  const [currentConversationId, setCurrentConversationId] = useState(null);
  const [connectionStatus, setConnectionStatus] = useState('checking'); // 'connected', 'disconnected', 'checking'
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    // Check connection status when widget opens
    if (isOpen) {
      const checkConnection = async () => {
        setConnectionStatus('checking');

        try {
          // Use the health check function to verify backend connectivity
          const isConnected = await ragApi.healthCheck();
          setConnectionStatus(isConnected ? 'connected' : 'disconnected');
        } catch (error) {
          console.error('Connection check failed:', error);
          setConnectionStatus('disconnected');
        }
      };

      checkConnection();
    }
  }, [isOpen]);

  const sendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      const data = await ragApi.chat(inputText, currentConversationId || null, 0.7);

      // Update conversation ID if this is a new conversation
      if (!currentConversationId) {
        setCurrentConversationId(data.conversation_id);
        setConversations(prev => [
          { id: data.conversation_id, title: inputText.substring(0, 30) + '...' },
          ...prev
        ]);
      }

      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        sources: data.sources || []
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, there was an error processing your request. Please try again.',
        sender: 'bot',
        isError: true
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const startNewConversation = async () => {
    setMessages([
      { id: 1, text: "Hello! I'm your RAG assistant. Ask me questions about the robotics book content.", sender: 'bot' }
    ]);
    setCurrentConversationId(null);

    try {
      const data = await ragApi.createConversation('New Conversation');
      setCurrentConversationId(data.conversation_id);
      setConversations(prev => [
        { id: data.conversation_id, title: 'New Conversation' },
        ...prev
      ]);
    } catch (error) {
      console.error('Error creating new conversation:', error);
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    sendMessage();
  };

  const formatSources = (sources) => {
    if (!sources || sources.length === 0) return null;

    return (
      <div className="sources">
        <strong>Sources:</strong> {sources.map((source, idx) =>
          source.content_preview ? `"${source.content_preview}"` : ''
        ).filter(Boolean).join(', ')}
      </div>
    );
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  return (
    <>
      {/* Floating Chat Button */}
      {!isOpen && (
        <button
          className="rag-chat-widget-button"
          onClick={toggleChat}
          aria-label="Open chat"
        >
          💬
        </button>
      )}

      {/* Chat Widget Container */}
      {isOpen && (
        <div className="rag-chat-widget-container">
          <div className="rag-chat-widget-header">
            <div style={{display: 'flex', justifyContent: 'space-between', alignItems: 'center', width: '100%'}}>
              <div>
                <h3>RAG Assistant</h3>
                <p>Ask about robotics book</p>
              </div>
              <div className={`connection-status ${connectionStatus}`}>
                <span className="status-indicator"></span>
                <span className="status-text">{connectionStatus === 'connected' ? 'Online' : connectionStatus === 'checking' ? 'Connecting...' : 'Offline'}</span>
              </div>
            </div>
            <button
              className="rag-chat-close-button"
              onClick={closeChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className="rag-chat-widget-content">
            <div className="rag-chat-messages">
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender}-message ${message.isError ? 'error-message' : ''}`}
                >
                  {message.text}
                  {message.sources && formatSources(message.sources)}
                </div>
              ))}
              {isLoading && (
                <div className="typing-indicator">
                  <div className="loading"></div>
                  <span>Thinking...</span>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>

            <div className="rag-chat-input-area">
              <form className="input-form" onSubmit={handleSubmit}>
                <input
                  type="text"
                  className="message-input"
                  value={inputText}
                  onChange={(e) => setInputText(e.target.value)}
                  placeholder="Ask about robotics..."
                  disabled={isLoading}
                  autoComplete="off"
                  required
                />
                <button
                  type="submit"
                  className="send-button"
                  disabled={isLoading || !inputText.trim()}
                >
                  Send
                </button>
              </form>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default RagChatWidget;