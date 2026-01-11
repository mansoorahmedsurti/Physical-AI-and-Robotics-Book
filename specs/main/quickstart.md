# RAG Chatbot - Quickstart Guide

## Prerequisites

- Python 3.11+
- Access to OpenAI API
- Qdrant Cloud account
- Neon Postgres account
- Node.js (for React widget integration)

## Setup

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up Backend Environment

#### Create and navigate to rag_backend directory:
```bash
cd rag_backend
```

#### Create virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

#### Install dependencies:
```bash
pip install -r requirements.txt
```

#### Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your API keys and connection strings
```

### 3. Environment Variables (.env)
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_HOST=your_qdrant_cloud_host
DATABASE_URL=postgresql://username:password@neon_host/dbname
POSTGRES_USER=username
POSTGRES_PASSWORD=password
POSTGRES_DB=rag_chatbot
SECRET_KEY=your_secret_key
DEBUG=False
```

## Running the Backend

### 1. Start the FastAPI Application
```bash
cd rag_backend
uvicorn main:app --reload --port 8000
```

### 2. Backend will be available at:
- API: http://localhost:8000
- Documentation: http://localhost:8000/docs
- Health: http://localhost:8000/health

## Document Ingestion

### 1. Ingest a document via API:
```bash
curl -X POST "http://localhost:8000/api/v1/documents/ingest" \
  -H "Content-Type: multipart/form-data" \
  -F "file=@path/to/your/document.txt" \
  -F "metadata={\"title\":\"Document Title\",\"author\":\"Author Name\"}"
```

### 2. Check ingestion status:
```bash
curl -X GET "http://localhost:8000/api/v1/documents/"
```

### 3. Delete a document:
```bash
curl -X DELETE "http://localhost:8000/api/v1/documents/{document_id}"
```

## Chat Interface

### 1. Start a conversation:
```bash
curl -X POST "http://localhost:8000/api/v1/chat/" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is this document about?",
    "temperature": 0.7
  }'
```

### 2. Continue conversation with conversation ID:
```bash
curl -X POST "http://localhost:8000/api/v1/chat/" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Can you elaborate on that point?",
    "conversation_id": "conversation-uuid",
    "temperature": 0.7
  }'
```

### 3. Start a new conversation:
```bash
curl -X POST "http://localhost:8000/api/v1/chat/conversation/" \
  -H "Content-Type: application/json" \
  -d '{}'
```

### 4. Get conversation history:
```bash
curl -X GET "http://localhost:8000/api/v1/chat/conversation/{conversation_id}"
```

## Health Checks

### 1. Health check:
```bash
curl -X GET "http://localhost:8000/health"
```

### 2. Readiness check:
```bash
curl -X GET "http://localhost:8000/ready"
```

## Frontend Integration

### 1. Install the React Chat Widget in your book application:
```bash
# Copy the react_widget directory to your frontend project
# Or create a package from it and install via npm
```

### 2. Import and use the widget:
```jsx
import React from 'react';
import RagChatWidget from './components/RagChatWidget';

function App() {
  return (
    <div className="App">
      <main>
        {/* Your existing book content */}
      </main>
      <RagChatWidget
        backendUrl="http://localhost:8000"
        documentContext="book-content"
      />
    </div>
  );
}

export default App;
```

## Testing

### 1. Run backend tests:
```bash
cd rag_backend
python -m pytest tests/
```

### 2. Run specific test suites:
```bash
# Unit tests
python -m pytest tests/unit/

# Integration tests
python -m pytest tests/integration/
```

## Production Deployment

### 1. Build Docker container:
```bash
cd rag_backend
docker build -t rag-chatbot-backend .
```

### 2. Run in production:
```bash
docker run -p 8000:8000 \
  -e OPENAI_API_KEY=your_key \
  -e QDRANT_HOST=your_host \
  -e DATABASE_URL=your_db_url \
  rag-chatbot-backend
```

## API Endpoints

### Documents
- `POST /api/v1/documents/ingest` - Ingest documents
- `GET /api/v1/documents/` - List documents
- `DELETE /api/v1/documents/{id}` - Delete document

### Chat
- `POST /api/v1/chat/` - Send message and get response
- `POST /api/v1/chat/conversation/` - Start new conversation
- `GET /api/v1/chat/conversation/{id}` - Get conversation history

### Health
- `GET /health` - Health check
- `GET /ready` - Readiness check

## Database Initialization

The application will automatically initialize the required database tables on first run. Make sure your Neon Postgres connection string is properly configured in the environment variables.