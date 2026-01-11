import os
import tempfile
from pathlib import Path
from fastapi import APIRouter, HTTPException, UploadFile, File, Form, Depends
from typing import Optional
from uuid import UUID, uuid4
from datetime import datetime
import hashlib
from ..models.document import Document, DocumentCreate, DocumentUpdate
from ..models.chat import IngestRequest, IngestResponse
from ..services.document_processor import document_processor
from ..services.qdrant_service import qdrant_service
from ..database import db
from ..utils import logger, InvalidDocumentException, DocumentNotFoundException
from ..auth import auth_service
from ..models.user import UserInDB

router = APIRouter()

@router.post("/documents/ingest", response_model=IngestResponse, tags=["documents"])
async def ingest_document(file: UploadFile = File(...), current_user: UserInDB = Depends(auth_service.get_current_user)):
    """
    Ingest a document for RAG processing
    """
    try:
        # Read the file content
        content = await file.read()

        # Determine file extension
        file_ext = Path(file.filename).suffix.lower()

        # For binary files (images, PDFs, DOCX), save temporarily and process
        if file_ext in ['.pdf', '.docx', '.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp']:
            # Save uploaded file temporarily
            temp_dir = tempfile.mkdtemp()
            temp_file_path = os.path.join(temp_dir, file.filename)

            with open(temp_file_path, 'wb') as temp_file:
                temp_file.write(content)

            try:
                # Process the file
                processed_chunks = await document_processor.process_file(temp_file_path)
            finally:
                # Clean up temporary file
                os.remove(temp_file_path)
                os.rmdir(temp_dir)

            # Decode content for storage in DB (just for metadata)
            content_str = "[Binary file processed]"
        else:
            # For text files, decode content normally
            content_str = content.decode('utf-8')

            # Process the document and generate chunks with embeddings
            processed_chunks = await document_processor.process_document(
                filename=file.filename,
                content=content_str
            )

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
        raise HTTPException(status_code=400, detail="File could not be decoded as text")
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
async def list_documents(limit: int = 10, offset: int = 0, status: Optional[str] = None, current_user: UserInDB = Depends(auth_service.get_current_user)):
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
async def delete_document(document_id: str, current_user: UserInDB = Depends(auth_service.get_current_user)):
    """
    Delete a document and its embeddings
    """
    try:
        # In a real implementation, this would delete from the database and check ownership
        # For now, just removing from Qdrant
        await qdrant_service.delete_document_chunks(document_id)

        logger.info(f"Successfully deleted document {document_id}")
        return {"message": f"Document {document_id} deleted successfully"}

    except Exception as e:
        logger.error(f"Error deleting document {document_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")