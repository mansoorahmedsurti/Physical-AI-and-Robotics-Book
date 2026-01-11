import asyncio
from qdrant_client import QdrantClient
from qdrant_client.http import models
from config import settings

async def reset_collection():
    # Connect to Qdrant
    client = QdrantClient(
        url=settings.QDRANT_HOST,
        api_key=settings.QDRANT_API_KEY,
    )

    collection_name = "document_chunks"

    # Drop the existing collection
    try:
        client.delete_collection(collection_name)
        print(f"Deleted collection: {collection_name}")
    except Exception as e:
        print(f"Collection {collection_name} may not have existed: {e}")

    # Recreate the collection with correct vector size for Cohere
    client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),  # Cohere embedding size
    )
    print(f"Created new collection: {collection_name} with vector size 1024")

    # Verify the collection was created
    collection_info = client.get_collection(collection_name)
    print(f"Collection info: {collection_info}")

if __name__ == "__main__":
    asyncio.run(reset_collection())