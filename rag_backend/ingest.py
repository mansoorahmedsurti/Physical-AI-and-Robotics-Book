import asyncio
import sys
import os
from pathlib import Path

# Add the rag_backend directory to the Python path
sys.path.insert(0, str(Path(__file__).parent))

from config import settings
from database import db
from services.document_processor import document_processor
from services.qdrant_service import qdrant_service

async def main():
    if len(sys.argv) != 2:
        print("Usage: python ingest.py <path_to_document>")
        sys.exit(1)

    document_path = sys.argv[1]
    if not os.path.exists(document_path):
        print(f"Error: File {document_path} does not exist")
        sys.exit(1)

    print(f"Starting ingestion of document: {document_path}")

    try:
        # Connect to database
        await db.connect()
        print("Connected to database")

        # Create document record in database
        file_size = os.path.getsize(document_path)
        file_ext = Path(document_path).suffix.lower()

        document_data = {
            'filename': os.path.basename(document_path),
            'original_name': os.path.basename(document_path),
            'content_type': f'application/{file_ext[1:]}' if file_ext else 'application/octet-stream',
            'size_bytes': file_size,
            'status': 'processing',
            'metadata': {}
        }

        document_id = await db.create_document(document_data)
        print(f"Created document record with ID: {document_id}")

        # Process the document and generate chunks with embeddings using the file path
        processed_chunks = await document_processor.process_file(document_path)
        print(f"Processed document into {len(processed_chunks)} chunks")

        # Store embeddings in Qdrant
        point_ids = await qdrant_service.store_embeddings(document_id, processed_chunks)
        print(f"Stored embeddings in Qdrant")

        # Update document status to completed
        await db.update_document_status(document_id, 'completed')
        print(f"Document {document_id} ingestion completed successfully")

    except Exception as e:
        print(f"Error during ingestion: {str(e)}")
        if 'document_id' in locals():
            await db.update_document_status(document_id, 'failed')
        sys.exit(1)
    finally:
        await db.disconnect()

if __name__ == "__main__":
    asyncio.run(main())