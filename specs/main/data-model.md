# RAG Chatbot - Data Models

## Database Schema (Neon Postgres)

### Documents Table
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

### Document Chunks Table
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
- **Vector Size**: 1024 (for Cohere embed-multilingual-v3.0)
- **Distance**: Cosine
- **Payload**:
  - `document_id`: UUID of source document
  - `chunk_index`: Index of chunk in document
  - `content_preview`: First 100 chars of content
  - `page_number`: Page number in original document (if applicable)

## Pydantic Data Models

### Document Models
```python
from pydantic import BaseModel
from typing import Optional, Dict, Any
from uuid import UUID
from datetime import datetime

class DocumentBase(BaseModel):
    filename: str
    original_name: str
    content_type: Optional[str] = None
    size_bytes: Optional[int] = None
    pages_count: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None

class DocumentCreate(DocumentBase):
    pass

class DocumentUpdate(BaseModel):
    status: Optional[str] = None
    processed_at: Optional[datetime] = None

class Document(DocumentBase):
    id: UUID
    uploaded_at: datetime
    processed_at: Optional[datetime] = None
    status: str

    class Config:
        from_attributes = True
```

### Chat Models
```python
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from uuid import UUID
from datetime import datetime

class MessageBase(BaseModel):
    conversation_id: UUID
    role: str
    content: str
    sources: Optional[List[Dict[str, Any]]] = None

class MessageCreate(MessageBase):
    pass

class Message(MessageBase):
    id: UUID
    timestamp: datetime

    class Config:
        from_attributes = True

class ConversationBase(BaseModel):
    user_id: Optional[str] = None
    title: Optional[str] = None

class ConversationCreate(ConversationBase):
    pass

class Conversation(ConversationBase):
    id: UUID
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

### Ingestion Models
```python
from pydantic import BaseModel
from typing import Optional, Dict, Any

class IngestRequest(BaseModel):
    filename: str
    content: str  # Document content as string
    metadata: Optional[Dict[str, Any]] = None

class IngestResponse(BaseModel):
    document_id: str
    filename: str
    chunks_created: int
    status: str

class QueryRequest(BaseModel):
    message: str
    conversation_id: Optional[str] = None
    temperature: Optional[float] = 0.7

class QueryResponse(BaseModel):
    conversation_id: str
    response: str
    sources: List[Dict[str, Any]]
    token_usage: Dict[str, int]
```

## API Contract Models

### Health Check Models
```python
from pydantic import BaseModel

class HealthResponse(BaseModel):
    status: str
    timestamp: str

class ReadyResponse(BaseModel):
    status: str
    services: Dict[str, bool]
```