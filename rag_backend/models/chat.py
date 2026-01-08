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