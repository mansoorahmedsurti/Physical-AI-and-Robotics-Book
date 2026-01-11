import asyncio
import sys
import os
from pathlib import Path
import importlib.util
import hashlib

# Add the rag_backend directory to the Python path
sys.path.insert(0, str(Path(__file__).parent))

def load_module_from_file(module_name, file_path):
    """Load a module from a file path"""
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

# Load modules directly from file paths
config = load_module_from_file("config", os.path.join(Path(__file__).parent, "config.py"))

# Load asyncpg separately for database operations
import asyncpg

async def connect_to_db():
    """Connect to the database using settings from config"""
    try:
        conn = await asyncpg.connect(
            dsn=config.settings.DATABASE_URL,
        )
        return conn
    except Exception as e:
        print(f"Failed to connect to database: {e}")
        raise

async def create_document_record(conn, document_data):
    """Create a document record in the database"""
    result = await conn.fetchrow("""
        INSERT INTO documents (
            filename, original_name, content_type, size_bytes,
            pages_count, checksum, metadata, status
        ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
        RETURNING id
    """,
    document_data.get('filename'),
    document_data.get('original_name'),
    document_data.get('content_type'),
    document_data.get('size_bytes'),
    document_data.get('pages_count'),
    document_data.get('checksum'),
    document_data.get('metadata'),
    document_data.get('status', 'pending')
    )

    return str(result['id'])

async def update_document_status(conn, document_id, status):
    """Update the status of a document"""
    await conn.execute("""
        UPDATE documents SET status = $1, processed_at = NOW() WHERE id = $2
    """, status, document_id)

async def main():
    if len(sys.argv) != 2:
        print("Usage: python simple_ingest.py <path_to_document_or_directory>")
        sys.exit(1)

    input_path = sys.argv[1]
    if not os.path.exists(input_path):
        print(f"Error: Path {input_path} does not exist")
        sys.exit(1)

    # Import document processor directly
    document_processor_module = load_module_from_file(
        "document_processor",
        os.path.join(Path(__file__).parent, "services", "document_processor.py")
    )
    document_processor = document_processor_module.document_processor

    # Import qdrant service directly
    qdrant_service_module = load_module_from_file(
        "qdrant_service",
        os.path.join(Path(__file__).parent, "services", "qdrant_service.py")
    )
    qdrant_service = qdrant_service_module.qdrant_service

    try:
        # Connect to database
        conn = await connect_to_db()
        print("Connected to database")

        # Check if it's a file or directory
        if os.path.isfile(input_path):
            # Process single file
            await process_single_file(conn, document_processor, qdrant_service, input_path)
        elif os.path.isdir(input_path):
            # Process directory
            await process_directory(conn, document_processor, qdrant_service, input_path)
        else:
            print(f"Error: {input_path} is neither a file nor a directory")
            sys.exit(1)

        await conn.close()
        print("Database connection closed")

    except Exception as e:
        print(f"Error during ingestion: {str(e)}")
        sys.exit(1)


async def process_single_file(conn, document_processor, qdrant_service, file_path):
    """Process a single file"""
    print(f"Starting ingestion of document: {file_path}")

    # Create document record in database
    file_size = os.path.getsize(file_path)
    file_ext = Path(file_path).suffix.lower()

    document_data = {
        'filename': os.path.basename(file_path),
        'original_name': os.path.basename(file_path),
        'content_type': f'application/{file_ext[1:]}' if file_ext else 'application/octet-stream',
        'size_bytes': file_size,
        'pages_count': None,  # Will be filled during processing for some file types
        'checksum': hashlib.md5(open(file_path, 'rb').read()).hexdigest(),
        'status': 'processing',
        'metadata': {}
    }

    document_id = await create_document_record(conn, document_data)
    print(f"Created document record with ID: {document_id}")

    try:
        # Process the document and generate chunks with embeddings using the file path
        processed_chunks = await document_processor.process_file(file_path)
        print(f"Processed document into {len(processed_chunks)} chunks")

        # Store embeddings in Qdrant
        point_ids = await qdrant_service.store_embeddings(document_id, processed_chunks)
        print(f"Stored embeddings in Qdrant")

        # Update document status to completed
        await update_document_status(conn, document_id, 'completed')
        print(f"Document {document_id} ingestion completed successfully")

    except Exception as e:
        print(f"Error processing document: {e}")
        await update_document_status(conn, document_id, 'failed')
        raise


async def process_directory(conn, document_processor, qdrant_service, directory_path):
    """Process all supported files in a directory"""
    print(f"Starting ingestion of directory: {directory_path}")

    # Supported file extensions
    supported_extensions = {
        '.pdf', '.docx', '.txt', '.md', '.html', '.htm',
        '.py', '.js', '.ts', '.java', '.cpp', '.c',
        '.json', '.xml', '.csv', '.jpg', '.jpeg', '.png', '.bmp'
    }

    # Find all supported files in the directory and subdirectories
    files_to_process = []
    for root, dirs, files in os.walk(directory_path):
        for file in files:
            file_path = os.path.join(root, file)
            if Path(file_path).suffix.lower() in supported_extensions:
                files_to_process.append(file_path)

    print(f"Found {len(files_to_process)} files to process")

    success_count = 0
    failure_count = 0

    for file_path in files_to_process:
        try:
            print(f"Processing file: {file_path}")
            await process_single_file(conn, document_processor, qdrant_service, file_path)
            success_count += 1
        except Exception as e:
            print(f"Error processing {file_path}: {str(e)}")
            failure_count += 1

    print(f"\nIngestion completed. Success: {success_count}, Failed: {failure_count}")


if __name__ == "__main__":
    asyncio.run(main())