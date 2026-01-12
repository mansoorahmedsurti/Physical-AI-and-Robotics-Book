import React, { useState, useEffect, useRef } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { ragApi } from '../utils/rag-api';
import '../css/rag-chat.css';

function RAGChatPage() {
  const { siteConfig } = useDocusaurusContext();
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your RAG assistant. Upload some documents and ask me questions about them.", sender: 'bot' }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversations, setConversations] = useState([]);
  const [currentConversationId, setCurrentConversationId] = useState(null);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

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
      { id: 1, text: "Hello! I'm your RAG assistant. Upload some documents and ask me questions about them.", sender: 'bot' }
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

  return (
    <Layout
      title={`RAG Chat - ${siteConfig.title}`}
      description="Interactive RAG chatbot for document querying">
      <div className="rag-chat-container">
        <div className="rag-chat-header">
          <h1>RAG Chatbot</h1>
          <p>Ask questions about your documents</p>
        </div>

        <div className="rag-chat-main">
          <div className="rag-chat-sidebar">
            <button className="new-conversation-btn" onClick={startNewConversation}>
              + New Conversation
            </button>
            <div className="conversations-list">
              {conversations.map((conv) => (
                <div
                  key={conv.id}
                  className={`conversation-item ${currentConversationId === conv.id ? 'active' : ''}`}
                  onClick={() => setCurrentConversationId(conv.id)}
                >
                  {conv.title}
                </div>
              ))}
              {conversations.length === 0 && (
                <div className="conversation-item">No conversations yet</div>
              )}
            </div>
          </div>

          <div className="rag-chat-area">
            <div className="messages-container">
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

            <div className="input-area">
              <form className="input-form" onSubmit={handleSubmit}>
                <input
                  type="text"
                  className="message-input"
                  value={inputText}
                  onChange={(e) => setInputText(e.target.value)}
                  placeholder="Type your message..."
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
      </div>
    </Layout>
  );
}

export default RAGChatPage;