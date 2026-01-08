# Basic test file for document processing
import pytest
from rag_backend.services.document_processor import DocumentProcessor

@pytest.mark.asyncio
async def test_document_chunking():
    processor = DocumentProcessor()
    text = "This is a test document. It has multiple sentences. We want to chunk it properly."

    chunks = processor._chunk_text(text, chunk_size=20, overlap=5)

    assert len(chunks) > 0
    assert 'content' in chunks[0]
    assert len(chunks[0]['content']) <= 20