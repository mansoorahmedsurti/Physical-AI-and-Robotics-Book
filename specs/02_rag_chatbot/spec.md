# 02 - RAG Chatbot Integration

## Feature Overview
Integrate a Retrieval-Augmented Generation (RAG) chatbot into the existing book repository as a monorepo. The RAG chatbot will allow users to ask questions about the book content and receive contextually relevant answers based on the book's documents.

## Scope

### In Scope
- RAG chatbot backend API using FastAPI
- Integration with Qdrant Cloud for vector storage
- Integration with Neon Serverless Postgres for metadata storage
- OpenAI API integration for question answering
- Document ingestion pipeline (PDF processing)
- React chat widget frontend component
- Separate deployment pipeline for backend (port 8000)
- Connection to existing book application (port 3000)

### Out of Scope
- Modification of existing book application files in src/
- Complete redesign of book application UI
- Migration of existing book data to new systems

## Technical Requirements

### Backend
- **Framework**: FastAPI (Python)
- **Server**: uvicorn
- **Environment**: python-dotenv

### Vector Database
- **Service**: Qdrant Cloud
- **Client**: qdrant-client
- **Functionality**: Store and retrieve document embeddings

### Relational Database
- **Service**: Neon Serverless Postgres
- **Driver**: psycopg2
- **Functionality**: Store document metadata, user sessions, conversation history

### AI Services
- **Provider**: OpenAI API
- **Models**: GPT-4 for question answering
- **Tokenization**: tiktoken for token counting

### Document Processing
- **Format**: PDF documents from book content
- **Processing**: Built-in Python libraries for PDF parsing
- **Embeddings**: OpenAI for text embeddings

### Frontend Integration
- **Component**: React Chat Widget
- **Communication**: REST API calls to backend
- **Styling**: Consistent with existing book application

## Functional Requirements

### Document Ingestion
1. Parse PDF documents from book content
2. Chunk documents into manageable pieces
3. Generate embeddings for each chunk
4. Store embeddings in Qdrant Cloud
5. Store metadata in Neon Postgres

### Question Answering
1. Accept user questions via API
2. Embed the question using the same model as documents
3. Retrieve relevant document chunks from Qdrant Cloud
4. Construct context from retrieved chunks
5. Generate answer using OpenAI API
6. Return answer with source references

### Conversation Management
1. Maintain conversation history
2. Handle follow-up questions
3. Track user sessions

## Non-Functional Requirements

### Performance
- Response time under 3 seconds for typical queries
- Support for 100 concurrent users
- 99.9% uptime for the RAG service

### Scalability
- Auto-scaling with Neon Serverless Postgres
- Efficient vector search with Qdrant Cloud
- Caching mechanisms for frequent queries

### Security
- Secure API keys management
- Rate limiting to prevent abuse
- Input sanitization to prevent injection attacks

### Reliability
- Proper error handling and graceful degradation
- Health checks and monitoring endpoints
- Backup and recovery procedures

## API Endpoints

### Document Management
- `POST /api/v1/documents/ingest` - Ingest documents
- `GET /api/v1/documents/` - List ingested documents
- `DELETE /api/v1/documents/{doc_id}` - Delete document

### Chat Interface
- `POST /api/v1/chat/` - Send message and get response
- `POST /api/v1/chat/conversation/` - Start new conversation
- `GET /api/v1/chat/conversation/{conv_id}` - Get conversation history

### Health Checks
- `GET /health` - Health check endpoint
- `GET /ready` - Readiness check endpoint

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

## Constraints

### Technical
- Backend must not interfere with existing book application
- All new code goes into rag_backend/ directory
- Existing src/ and docs/ files remain unchanged
- Use of specified tech stack is mandatory

### Resource
- Must work within Qdrant Cloud and Neon Serverless Postgres limitations
- OpenAI API usage should be optimized for cost efficiency

## Assumptions

- Book content is available in PDF format
- OpenAI API access is properly configured
- Qdrant Cloud and Neon Postgres credentials are available
- Existing book application can accommodate frontend widget integration