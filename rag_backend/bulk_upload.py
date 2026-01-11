import os
from pathlib import Path
from typing import List, Dict, Any
from .services.document_processor import document_processor
from .services.qdrant_service import qdrant_service
from .database import db
from .utils import logger
import hashlib
import asyncio


class BulkUploader:
    def __init__(self):
        self.supported_extensions = {
            '.pdf', '.docx', '.txt', '.md', '.html', '.htm',
            '.py', '.js', '.ts', '.java', '.cpp', '.c',
            '.json', '.xml', '.csv', '.jpg', '.jpeg', '.png', '.bmp'
        }

    async def upload_directory(self, directory_path: str) -> Dict[str, Any]:
        """
        Upload all supported files from a directory to the RAG system
        """
        directory = Path(directory_path)

        if not directory.exists():
            raise FileNotFoundError(f"Directory does not exist: {directory_path}")

        if not directory.is_dir():
            raise NotADirectoryError(f"Path is not a directory: {directory_path}")

        # Find all supported files
        files_to_process = []
        for ext in self.supported_extensions:
            files_to_process.extend(directory.glob(f"**/*{ext}"))
            files_to_process.extend(directory.glob(f"**/*{ext.upper()}"))

        results = {
            "total_files": len(files_to_process),
            "successful": 0,
            "failed": 0,
            "processed_files": [],
            "errors": []
        }

        logger.info(f"Found {len(files_to_process)} files to process")

        for file_path in files_to_process:
            try:
                logger.info(f"Processing file: {file_path}")

                # Process the file
                processed_chunks = await document_processor.process_file(str(file_path))

                # Generate document ID based on file path
                file_id = hashlib.md5(str(file_path).encode()).hexdigest()

                # Store embeddings in Qdrant
                point_ids = await qdrant_service.store_embeddings(file_id, processed_chunks)

                # Store document metadata in database
                file_size = file_path.stat().st_size
                with open(file_path, 'rb') as f:
                    file_checksum = hashlib.sha256(f.read()).hexdigest()

                document_data = {
                    'filename': file_path.name,
                    'original_name': str(file_path),
                    'content_type': file_path.suffix.lower(),
                    'size_bytes': file_size,
                    'checksum': file_checksum,
                    'status': 'completed',
                    'metadata': {
                        'source_path': str(file_path),
                        'processed_at': asyncio.get_event_loop().time()
                    }
                }

                document_id = await db.create_document(document_data)

                results["processed_files"].append({
                    "file_path": str(file_path),
                    "document_id": document_id,
                    "chunks_created": len(processed_chunks),
                    "status": "success"
                })

                results["successful"] += 1
                logger.info(f"Successfully processed: {file_path}")

            except Exception as e:
                error_msg = f"Error processing {file_path}: {str(e)}"
                logger.error(error_msg)
                results["errors"].append(error_msg)
                results["failed"] += 1

        return results

    async def upload_single_file(self, file_path: str) -> Dict[str, Any]:
        """
        Upload a single file to the RAG system
        """
        file_path = Path(file_path)

        if not file_path.exists():
            raise FileNotFoundError(f"File does not exist: {file_path}")

        if file_path.suffix.lower() not in self.supported_extensions:
            raise ValueError(f"Unsupported file type: {file_path.suffix}. Supported types: {self.supported_extensions}")

        try:
            # Process the file
            processed_chunks = await document_processor.process_file(str(file_path))

            # Generate document ID based on file path
            file_id = hashlib.md5(str(file_path).encode()).hexdigest()

            # Store embeddings in Qdrant
            point_ids = await qdrant_service.store_embeddings(file_id, processed_chunks)

            # Store document metadata in database
            file_size = file_path.stat().st_size
            with open(file_path, 'rb') as f:
                file_checksum = hashlib.sha256(f.read()).hexdigest()

            document_data = {
                'filename': file_path.name,
                'original_name': str(file_path),
                'content_type': file_path.suffix.lower(),
                'size_bytes': file_size,
                'checksum': file_checksum,
                'status': 'completed',
                'metadata': {
                    'source_path': str(file_path),
                    'processed_at': asyncio.get_event_loop().time()
                }
            }

            document_id = await db.create_document(document_data)

            result = {
                "file_path": str(file_path),
                "document_id": document_id,
                "chunks_created": len(processed_chunks),
                "status": "success",
                "message": f"Successfully uploaded {file_path.name}"
            }

            logger.info(f"Successfully processed: {file_path}")
            return result

        except Exception as e:
            error_msg = f"Error processing {file_path}: {str(e)}"
            logger.error(error_msg)
            raise


# Global bulk uploader instance
bulk_uploader = BulkUploader()