# RAG Chatbot - Data Model

## Document Schema

### Document Metadata
```sql
CREATE TABLE documents (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    filename VARCHAR(255) NOT NULL,
    original_name VARCHAR(255) NOT NULL,
    content_type VARCHAR(50),
    size_bytes INTEGER,
    pages_count INTEGER,
    checksum VARCHAR(64),
    uploaded_at TIMESTAMP DEFAULT NOW(),
    processed_at TIMESTAMP,
    status VARCHAR(20) DEFAULT 'pending' CHECK (status IN ('pending', 'processing', 'completed', 'failed')),
    metadata JSONB
);
```

### Chunks Table
```sql
CREATE TABLE document_chunks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    document_id UUID REFERENCES documents(id) ON DELETE CASCADE,
    chunk_index INTEGER NOT NULL,
    content TEXT NOT NULL,
    token_count INTEGER,
    embedding_id VARCHAR(255), -- Reference to Qdrant point ID
    created_at TIMESTAMP DEFAULT NOW()
);
```

### Conversations Table
```sql
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(255),
    title VARCHAR(255),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);
```

### Messages Table
```sql
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID REFERENCES conversations(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    sources JSONB, -- Array of document references
    timestamp TIMESTAMP DEFAULT NOW()
);
```

## Qdrant Collection Schema

### Document Chunks Collection
- **Collection Name**: `document_chunks`
- **Vector Size**: Determined by embedding model (e.g., 1536 for text-embedding-ada-002)
- **Distance**: Cosine
- **Payload**:
  - `document_id`: UUID of source document
  - `chunk_index`: Index of chunk in document
  - `content_preview`: First 100 chars of content
  - `page_number`: Page number in original document (if applicable)

## API Data Structures

### Ingest Request
```json
{
  "file": "binary file data",
  "metadata": {
    "source": "book_content",
    "title": "Document Title",
    "author": "Author Name"
  }
}
```

### Ingest Response
```json
{
  "document_id": "uuid",
  "filename": "original_filename.pdf",
  "pages_processed": 10,
  "chunks_created": 25,
  "status": "processing"
}
```

### Chat Request
```json
{
  "message": "What is the main concept discussed in chapter 1?",
  "conversation_id": "optional uuid",
  "temperature": 0.7
}
```

### Chat Response
```json
{
  "conversation_id": "uuid",
  "response": "The main concept discussed in chapter 1 is...",
  "sources": [
    {
      "document_id": "uuid",
      "filename": "document.pdf",
      "page_number": 5,
      "relevance_score": 0.87
    }
  ],
  "token_usage": {
    "prompt_tokens": 120,
    "completion_tokens": 85,
    "total_tokens": 205
  }
}
```