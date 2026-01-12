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
  const [currentPageContext, setCurrentPageContext] = useState('');
  const messagesEndRef = useRef(null);

  // Determine context based on current URL
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const path = window.location.pathname;

      if (path.includes('/docs/ros')) {
        setCurrentPageContext('ROS 2');
      } else if (path.includes('/docs/isaac') || path.includes('/docs/simulation')) {
        setCurrentPageContext('Isaac Sim');
      } else if (path.includes('/docs/edge') || path.includes('/docs/deployment')) {
        setCurrentPageContext('Edge Deployment');
      } else if (path.includes('/docs/vla') || path.includes('/docs/models')) {
        setCurrentPageContext('VLA Models');
      } else if (path.includes('/docs/rl') || path.includes('/docs/training')) {
        setCurrentPageContext('RL Training');
      } else if (path.includes('/docs/sim2real')) {
        setCurrentPageContext('Sim2Real Gap');
      } else {
        setCurrentPageContext('General');
      }
    }
  }, []);

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

  // Get suggested prompts based on current context
  const getSuggestedPrompts = () => {
    const prompts = {
      'ROS 2': [
        "How do I create a ROS 2 node?",
        "What are the communication patterns in ROS 2?",
        "How to use ROS 2 actions and services?"
      ],
      'Isaac Sim': [
        "How do I create a robot in Isaac Sim?",
        "What are the physics parameters in Isaac Sim?",
        "How to use OmniGibson with Isaac Sim?"
      ],
      'Edge Deployment': [
        "How to optimize models for Jetson Orin?",
        "What are the deployment strategies for edge AI?",
        "How to reduce latency on edge devices?"
      ],
      'VLA Models': [
        "How do Vision-Language-Action models work?",
        "What are the input requirements for VLA models?",
        "How to fine-tune VLA models?"
      ],
      'RL Training': [
        "How to implement PPO for robot control?",
        "What are good RL training practices?",
        "How to use OmniIsaacGymEnvs?"
      ],
      'Sim2Real Gap': [
        "How to bridge the sim-to-real gap?",
        "What domain randomization techniques work?",
        "How to make sim policies work in reality?"
      ],
      'General': [
        "What is embodied intelligence?",
        "How do robots learn from demonstration?",
        "What are the latest trends in humanoid robotics?"
      ]
    };

    return prompts[currentPageContext] || prompts['General'];
  };

  const handleSuggestedPromptClick = (prompt) => {
    setInputText(prompt);
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
                <p>Ask about {currentPageContext} robotics</p>
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

            {/* Suggested prompts based on current page context */}
            <div className="suggested-prompts">
              {getSuggestedPrompts().slice(0, 3).map((prompt, index) => (
                <button
                  key={index}
                  className="suggested-prompt-btn"
                  onClick={() => handleSuggestedPromptClick(prompt)}
                  disabled={isLoading}
                >
                  {prompt}
                </button>
              ))}
            </div>

            <div className="rag-chat-input-area">
              <form className="input-form" onSubmit={handleSubmit}>
                <input
                  type="text"
                  className="message-input"
                  value={inputText}
                  onChange={(e) => setInputText(e.target.value)}
                  placeholder={`Ask about ${currentPageContext}...`}
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