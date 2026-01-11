import sys
import os
import asyncio
from pathlib import Path
import platform

# ==========================================
# 🚑 WINDOWS CRASH FIX (Keep this!)
# ==========================================
def fake_win32_ver(*args, **kwargs):
    return ('10', '10.0.19041', 'SP0', 'Multiprocessor Free')
platform.win32_ver = fake_win32_ver
# ==========================================

# Setup paths
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from database import db
    from services.document_processor import document_processor
    from services.qdrant_service import qdrant_service
except ImportError as e:
    print(f"❌ CRITICAL IMPORT ERROR: {e}")
    sys.exit(1)

async def process_directory_sequentially(directory_path):
    print(f"\n📂 Scanning directory: {directory_path}")
    
    ignore_extensions = ['.py', '.pyc', '.js', '.json', '.css', '.html', '.env', '.gitignore']
    files_to_process = []
    
    # 1. Gather all files first
    for root, dirs, files in os.walk(directory_path):
        for file in files:
            file_path = os.path.join(root, file)
            file_ext = Path(file_path).suffix.lower()
            
            if file_ext in ignore_extensions: continue
            valid_exts = ['.md', '.txt', '.pdf', '.docx']
            if file_ext not in valid_exts: continue
            
            files_to_process.append(file_path)

    if not files_to_process:
        print("❌ No valid documents found!")
        return

    print(f"\n🚀 Found {len(files_to_process)} documents. Processing one by one...")

    # 2. Process SEQUENTIALLY (One by one) to save the DB connection
    for file_path in files_to_process:
        await process_single_file(file_path)
        # Small pause to let the database breathe
        await asyncio.sleep(0.5)

async def process_single_file(file_path):
    # SMARTER NAMING: Combine Folder + Filename
    # Example: "Intro/README.md" -> "Intro_README.md"
    folder_name = os.path.basename(os.path.dirname(file_path))
    base_name = os.path.basename(file_path)
    unique_filename = f"{folder_name}_{base_name}"
    
    print(f"   -------------------------------------------------")
    print(f"   📄 Processing: {unique_filename}")

    try:
        file_size = os.path.getsize(file_path)
        
        # 1. Create Record in Neon DB
        document_data = {
            'filename': unique_filename, # Use the unique name!
            'original_name': base_name,
            'content_type': 'text/markdown',
            'size_bytes': file_size,
            'status': 'processing',
            'metadata': {"source": "local_ingest"}
        }
        
        # Connect strictly for this operation to avoid stale connections
        if not db.pool: await db.connect()
        
        document_id = await db.create_document(document_data)
        print(f"      🔹 DB Record Created (ID: {document_id})")
        
        # 2. Chunking
        print(f"      🔹 Chunking text...")
        chunks = await document_processor.process_file(file_path)
        
        # 3. Upload to Qdrant
        print(f"      🔹 Uploading {len(chunks)} chunks to Qdrant...")
        await qdrant_service.store_embeddings(document_id, chunks)
        
        # 4. Finish
        await db.update_document_status(document_id, 'completed')
        print(f"   ✅ SUCCESS: {unique_filename}")

    except Exception as e:
        print(f"   ❌ FAILED: {unique_filename}") 
        print(f"      Reason: {str(e)}")

async def main():
    print("🔌 Connecting to Database...")
    await db.connect()
    print("✅ Database Connected.")

    if len(sys.argv) < 2:
        print("Usage: python ingest.py <folder_path>")
        return

    target_path = sys.argv[1].strip('"').strip("'")

    if os.path.isdir(target_path):
        await process_directory_sequentially(target_path)
    else:
        await process_single_file(target_path)

    await db.disconnect()
    print("\n👋 All Done.")

if __name__ == "__main__":
    asyncio.run(main())