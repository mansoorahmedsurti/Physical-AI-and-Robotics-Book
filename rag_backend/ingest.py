import sys
import os
import asyncio
from pathlib import Path
import platform

# ==========================================
# 🚑 CRITICAL FIX FOR YOUR BROKEN WINDOWS
# ==========================================
# This function forces Python to STOP asking Windows for its version.
# This prevents the freeze/crash you are seeing.
def fake_win32_ver(*args, **kwargs):
    return ('10', '10.0.19041', 'SP0', 'Multiprocessor Free')

platform.win32_ver = fake_win32_ver
# ==========================================

# Setup paths
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from config import settings
    from database import db
    from services.document_processor import document_processor
    from services.qdrant_service import qdrant_service
except ImportError as e:
    print(f"❌ CRITICAL IMPORT ERROR: {e}")
    sys.exit(1)

async def process_directory(directory_path):
    print(f"\n📂 Scanning directory: {directory_path}")
    
    ignore_extensions = ['.py', '.pyc', '.js', '.json', '.css', '.html', '.env', '.gitignore']
    tasks = []
    
    for root, dirs, files in os.walk(directory_path):
        for file in files:
            file_path = os.path.join(root, file)
            file_ext = Path(file_path).suffix.lower()
            
            # Filter for valid docs
            if file_ext in ignore_extensions: continue
            valid_exts = ['.md', '.txt', '.pdf', '.docx']
            if file_ext not in valid_exts: continue
                
            print(f"   found: {file}")
            tasks.append(process_single_file(file_path))

    if not tasks:
        print("❌ No valid documents found!")
        return

    print(f"\n🚀 Found {len(tasks)} documents. Starting ingestion...")
    await asyncio.gather(*tasks)

async def process_single_file(file_path):
    filename = os.path.basename(file_path)
    try:
        file_size = os.path.getsize(file_path)
        
        # 1. Neon DB
        print(f"   Creating DB record for {filename}...")
        document_data = {
            'filename': filename,
            'original_name': filename,
            'content_type': 'text/markdown',
            'size_bytes': file_size,
            'status': 'processing',
            'metadata': {"source": "local_ingest"}
        }
        document_id = await db.create_document(document_data)
        
        # 2. Chunking
        print(f"   Chunking {filename}...")
        chunks = await document_processor.process_file(file_path)
        
        # 3. Qdrant
        print(f"   Uploading {filename} to Qdrant...")
        await qdrant_service.store_embeddings(document_id, chunks)
        
        # 4. Finish
        await db.update_document_status(document_id, 'completed')
        print(f"✅ COMPLETED: {filename}")

    except Exception as e:
        print(f"❌ FAILED: {filename} | Error: {str(e)}")

async def main():
    print("🔍 DEBUG: Starting manual ingestion...")
    try:
        await db.connect()
        print("✅ Database Connected!")
    except Exception as e:
        print(f"❌ DATABASE ERROR: {e}")
        return

    if len(sys.argv) < 2:
        print("Usage: python ingest.py <folder_path>")
        return

    target_path = sys.argv[1]
    target_path = target_path.strip('"').strip("'")

    if os.path.isdir(target_path):
        await process_directory(target_path)
    else:
        await process_single_file(target_path)

    await db.disconnect()
    print("👋 Done.")

if __name__ == "__main__":
    asyncio.run(main())