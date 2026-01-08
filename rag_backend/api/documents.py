from fastapi import APIRouter, HTTPException, UploadFile, File, Form
from typing import Optional
from uuid import UUID, uuid4
from datetime import datetime
import hashlib
from ..models.document import Document, DocumentCreate, DocumentUpdate, IngestRequest, IngestResponse
from ..services.document_processor import document_processor
from ..services.qdrant_service import qdrant_service
from ..database import db
from ..utils import logger, InvalidDocumentException, DocumentNotFoundException

router = APIRouter()

@router.post("/documents/ingest", response_model=IngestResponse, tags=["documents"])
async def ingest_document(file: UploadFile = File(...)):
    """
    Ingest a document for RAG processing
    """
    try:
        # Read the file content
        content = await file.read()
        content_str = content.decode('utf-8')

        # Calculate file size and checksum
        size_bytes = len(content)
        checksum = hashlib.sha256(content).hexdigest()

        # Create document record in database
        document_data = {
            'filename': file.filename,
            'original_name': file.filename,
            'content_type': file.content_type,
            'size_bytes': size_bytes,
            'checksum': checksum,
            'status': 'processing',
            'metadata': {}
        }

        document_id = await db.create_document(document_data)

        # Process the document and generate chunks with embeddings
        processed_chunks = await document_processor.process_document(
            filename=file.filename,
            content=content_str
        )

        # Store embeddings in Qdrant
        point_ids = await qdrant_service.store_embeddings(document_id, processed_chunks)

        # Update document status to completed
        await db.update_document_status(document_id, 'completed')

        # Prepare response
        response = IngestResponse(
            document_id=document_id,
            filename=file.filename,
            chunks_created=len(processed_chunks),
            status="completed"
        )

        logger.info(f"Successfully ingested document {document_id} with {len(processed_chunks)} chunks")
        return response

    except UnicodeDecodeError:
        # Update document status to failed
        if 'document_id' in locals():
            await db.update_document_status(document_id, 'failed')
        raise HTTPException(status_code=400, detail="File must be a text file")
    except InvalidDocumentException as e:
        # Update document status to failed
        if 'document_id' in locals():
            await db.update_document_status(document_id, 'failed')
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        # Update document status to failed
        if 'document_id' in locals():
            await db.update_document_status(document_id, 'failed')
        logger.error(f"Error ingesting document: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/documents/", response_model=dict, tags=["documents"])
async def list_documents(limit: int = 10, offset: int = 0, status: Optional[str] = None):
    """
    List all ingested documents
    """
    try:
        documents_list = await db.list_documents(limit, offset, status)

        # Count total documents for pagination
        total_query = "SELECT COUNT(*) FROM documents"
        total_params = []
        if status:
            total_query += " WHERE status = $1"
            total_params.append(status)

        total_result = await db.execute_query(total_query, *total_params)
        total = total_result[0]['count']

        response = {
            "documents": documents_list,
            "total": total,
            "limit": limit,
            "offset": offset
        }

        return response
    except Exception as e:
        logger.error(f"Error listing documents: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.delete("/documents/{document_id}", tags=["documents"])
async def delete_document(document_id: str):
    """
    Delete a document and its embeddings
    """
    try:
        # In a real implementation, this would delete from the database
        # For now, just removing from Qdrant
        await qdrant_service.delete_document_chunks(document_id)

        logger.info(f"Successfully deleted document {document_id}")
        return {"message": f"Document {document_id} deleted successfully"}

    except Exception as e:
        logger.error(f"Error deleting document {document_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")