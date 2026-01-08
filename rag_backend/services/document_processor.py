import asyncio
import tempfile
from typing import List, Dict, Any, Tuple
from pathlib import Path
import PyPDF2
from ..utils import logger, InvalidDocumentException
from ..services.qdrant_service import qdrant_service
from ..services.embedding_service import EmbeddingService
from ..config import settings

class DocumentProcessor:
    def __init__(self):
        self.embedding_service = EmbeddingService()

    async def process_document(self, filename: str, content: str) -> List[Dict[str, Any]]:
        """
        Process document content and return chunks with embeddings
        """
        try:
            # For now, we'll split the content into chunks
            # In a real implementation, we'd use proper document parsing libraries
            chunks = self._chunk_text(content)

            # Generate embeddings for each chunk
            chunk_embeddings = await self.embedding_service.generate_embeddings([chunk['content'] for chunk in chunks])

            # Combine chunks with their embeddings
            processed_chunks = []
            for i, chunk in enumerate(chunks):
                processed_chunk = {
                    'content': chunk['content'],
                    'embedding': chunk_embeddings[i],
                    'page_number': chunk.get('page_number', 1)
                }
                processed_chunks.append(processed_chunk)

            return processed_chunks
        except Exception as e:
            logger.error(f"Error processing document: {str(e)}")
            raise InvalidDocumentException(f"Could not process document: {str(e)}")

    def _chunk_text(self, text: str, chunk_size: int = 1000, overlap: int = 100) -> List[Dict[str, Any]]:
        """
        Split text into overlapping chunks
        """
        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size

            # If we're near the end, extend to include the rest
            if end > len(text):
                end = len(text)

            chunk_text = text[start:end]
            chunks.append({
                'content': chunk_text,
                'start_pos': start,
                'end_pos': end
            })

            # Move start position forward, accounting for overlap
            start = end - overlap

            # If the next chunk would be empty, break
            if start >= len(text):
                break

        return chunks

    async def process_file(self, file_path: str) -> List[Dict[str, Any]]:
        """
        Process a file from disk and return chunks with embeddings
        """
        try:
            file_extension = Path(file_path).suffix.lower()

            if file_extension == '.pdf':
                content = self._extract_text_from_pdf(file_path)
            elif file_extension in ['.txt', '.md', '.py', '.js', '.html', '.css']:
                with open(file_path, 'r', encoding='utf-8') as file:
                    content = file.read()
            else:
                # For other files, try to read as text
                with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
                    content = file.read()

            return await self.process_document(Path(file_path).name, content)
        except Exception as e:
            logger.error(f"Error processing file {file_path}: {str(e)}")
            raise InvalidDocumentException(f"Could not process file {file_path}: {str(e)}")

    def _extract_text_from_pdf(self, pdf_path: str) -> str:
        """
        Extract text from a PDF file
        """
        text = ""
        try:
            with open(pdf_path, 'rb') as file:
                pdf_reader = PyPDF2.PdfReader(file)
                for page in pdf_reader.pages:
                    text += page.extract_text() + "\n"
        except Exception as e:
            logger.error(f"Error extracting text from PDF {pdf_path}: {str(e)}")
            raise
        return text

# Global document processor instance
document_processor = DocumentProcessor()