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