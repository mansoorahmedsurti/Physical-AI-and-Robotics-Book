import logging
from fastapi import HTTPException, status
from typing import Any, Dict

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_logger(name: str) -> logging.Logger:
    """Get a logger with the specified name"""
    return logging.getLogger(name)

# Custom exceptions
class RagChatbotException(HTTPException):
    def __init__(self, detail: Any = None, status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR):
        super().__init__(status_code=status_code, detail=detail)

class DocumentNotFoundException(RagChatbotException):
    def __init__(self, document_id: str):
        super().__init__(
            detail=f"Document with id {document_id} not found",
            status_code=status.HTTP_404_NOT_FOUND
        )

class InvalidDocumentException(RagChatbotException):
    def __init__(self, detail: str):
        super().__init__(
            detail=f"Invalid document: {detail}",
            status_code=status.HTTP_400_BAD_REQUEST
        )

class EmbeddingGenerationException(RagChatbotException):
    def __init__(self, detail: str):
        super().__init__(
            detail=f"Error generating embeddings: {detail}",
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR
        )

class DatabaseConnectionException(RagChatbotException):
    def __init__(self, detail: str):
        super().__init__(
            detail=f"Database connection error: {detail}",
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR
        )