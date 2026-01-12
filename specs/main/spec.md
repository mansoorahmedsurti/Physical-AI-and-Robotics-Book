# Integrated RAG Chatbot Development

## Feature Overview
Build and embed a Retrieval-Augmented Generation (RAG) chatbot into the published book that can answer questions using full-book context or user-selected text only.

## Scope

### In Scope
- FastAPI backend (rag_backend/, port 8000)
- Cohere integration (Command-R Plus model)
- Qdrant Cloud (vector storage)
- Neon Serverless Postgres (metadata, sessions, history)
- PDF ingestion, chunking, embeddings
- React chat widget integration with book app (port 3000)

### Out of Scope
- Modifying existing src/ or docs/
- Redesigning book UI
- Migrating existing book data

## Technical Architecture

### Backend
- FastAPI + uvicorn server
- REST APIs for documents, chat, and health checks
- RAG pipeline using Cohere (embed → retrieve → generate)
- Embedding model: embed-english-v3.0
- Chat model: command-r
- No local model fallback allowed due to 8GB RAM constraint

### Storage
- Qdrant Cloud: embeddings and similarity search
- Neon Serverless Postgres: documents, metadata, conversations

### Frontend
- Standalone React chat widget
- REST communication with backend

## Functional Requirements

### Document Processing
- Parse PDF documents from book content
- Chunk documents into searchable segments
- Generate embeddings for each chunk using Cohere embed-english-v3.0 model
- Store embeddings in Qdrant Cloud
- Store metadata in Neon Postgres

### Chat Functionality
- Accept user questions about book content
- Retrieve relevant document chunks from Qdrant
- Generate context-aware answers using Cohere command-r model
- Provide source citations for answers
- Maintain conversation history
- No local model fallback allowed due to 8GB RAM constraint

### Integration
- Embed React chat widget into existing book application
- Maintain separation between existing app and new functionality
- Ensure seamless user experience

## Non-Functional Requirements

### Performance
- Response time under 3 seconds for 95% of typical queries
- Support for 100 concurrent users with load testing validation
- 99.9% uptime for the RAG service measured monthly
- System must handle 1000 requests per minute sustained load

### Scalability
- Auto-scaling with Neon Serverless Postgres
- Efficient vector search with Qdrant Cloud
- Caching mechanisms for frequent queries

### Security
- Secure API keys management
- Rate limiting to prevent abuse
- Input sanitization to prevent injection attacks

### Accuracy Verification
- Implement source citation in responses to allow verification of information
- Include confidence scores in responses to indicate response reliability
- Log and monitor response quality metrics for continuous improvement

## Success Criteria

### Primary
- Users can ask questions about book content and receive accurate answers
- RAG system can handle book documents in PDF format
- Backend runs independently on port 8000
- Frontend widget integrates seamlessly with existing book app

### Secondary
- Response quality meets user expectations
- System handles concurrent users effectively
- Proper error handling and user feedback