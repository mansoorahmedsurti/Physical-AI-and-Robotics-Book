# RAG Chatbot API Contracts

## Document Ingestion API

### POST /api/v1/documents/ingest
**Summary**: Ingest a document for RAG processing

**Request**:
- Content-Type: multipart/form-data
- Body: file (binary), metadata (JSON string)

**Responses**:
- 200: Document ingestion initiated successfully
  - Content-Type: application/json
  - Body: `{ "document_id": "uuid", "status": "processing", "message": "Document queued for processing" }`
- 400: Invalid request
- 500: Internal server error

### GET /api/v1/documents/
**Summary**: List all ingested documents

**Request**:
- Query params: limit (int), offset (int), status (string)

**Responses**:
- 200: List of documents
  - Content-Type: application/json
  - Body: `{ "documents": [{"id": "uuid", "filename": "name", "status": "completed", ...}], "total": 5 }`
- 500: Internal server error

### DELETE /api/v1/documents/{id}
**Summary**: Delete a document and its embeddings

**Path Parameters**:
- id: Document UUID

**Responses**:
- 200: Document deleted successfully
- 404: Document not found
- 500: Internal server error

## Chat API

### POST /api/v1/chat/
**Summary**: Send a message and get a RAG-enhanced response

**Request**:
- Content-Type: application/json
- Body: `{ "message": "user question", "conversation_id": "optional uuid", "temperature": 0.7 }`

**Responses**:
- 200: Chat response with sources
  - Content-Type: application/json
  - Body: `{ "conversation_id": "uuid", "response": "answer", "sources": [...], "token_usage": {...} }`
- 400: Invalid request
- 500: Internal server error

### POST /api/v1/chat/conversation/
**Summary**: Start a new conversation

**Request**:
- Content-Type: application/json
- Body: `{ "title": "optional title" }`

**Responses**:
- 200: New conversation created
  - Content-Type: application/json
  - Body: `{ "conversation_id": "uuid", "title": "title", "created_at": "timestamp" }`
- 400: Invalid request
- 500: Internal server error

## Health API

### GET /health
**Summary**: Health check for the service

**Responses**:
- 200: Service healthy
  - Content-Type: application/json
  - Body: `{ "status": "healthy", "timestamp": "iso8601" }`
- 503: Service unhealthy

### GET /ready
**Summary**: Readiness check for the service

**Responses**:
- 200: Service ready
  - Content-Type: application/json
  - Body: `{ "status": "ready", "services": {"database": true, "qdrant": true, "cohere": true} }`
- 503: Service not ready