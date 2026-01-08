from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
from ..config import settings

class QdrantService:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_HOST,
            api_key=settings.QDRANT_API_KEY,
        )
        self.collection_name = "document_chunks"
        self._initialize_collection()

    def _initialize_collection(self):
        """Initialize the document chunks collection if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # OpenAI embedding size
            )
            print(f"Created collection: {self.collection_name}")

    async def store_embeddings(self, document_id: str, chunks: List[Dict[str, Any]]) -> List[str]:
        """Store document chunks with their embeddings in Qdrant"""
        points = []
        for idx, chunk in enumerate(chunks):
            point = models.PointStruct(
                id=f"{document_id}_{idx}",
                vector=chunk['embedding'],
                payload={
                    "document_id": document_id,
                    "chunk_index": idx,
                    "content": chunk['content'],
                    "content_preview": chunk['content'][:100],
                    "page_number": chunk.get('page_number', 1)
                }
            )
            points.append(point)

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        return [point.id for point in points]

    async def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """Search for similar document chunks based on query embedding"""
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True
        )

        results = []
        for hit in search_result:
            results.append({
                "document_id": hit.payload["document_id"],
                "chunk_index": hit.payload["chunk_index"],
                "content": hit.payload["content"],
                "score": hit.score,
                "page_number": hit.payload.get("page_number", 1)
            })

        return results

    async def delete_document_chunks(self, document_id: str):
        """Delete all chunks associated with a document"""
        # Find all points with this document_id
        scroll_result = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="document_id",
                        match=models.MatchValue(value=document_id)
                    )
                ]
            ),
            limit=10000  # Assuming reasonable upper limit for document chunks
        )

        point_ids = [hit.id for hit in scroll_result[0]]

        if point_ids:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=point_ids)
            )

# Global Qdrant service instance
qdrant_service = QdrantService()