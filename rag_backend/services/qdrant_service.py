from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
from config import settings

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

    async def search_similar(self, query_embedding: List[float], limit: int = 5, query_text: str = None) -> List[Dict[str, Any]]:
        """Search for similar document chunks based on query embedding"""
        # If query_text is provided, perform hybrid search (semantic + keyword)
        if query_text:
            search_result = self._hybrid_search(query_embedding, query_text, limit)
        else:
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

        # Re-rank results if we have query text
        if query_text:
            results = self._re_rank_results(results, query_text)

        return results

    def _hybrid_search(self, query_embedding: List[float], query_text: str, limit: int) -> List:
        """Perform hybrid search combining semantic and keyword search"""
        # Perform dense vector search
        semantic_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit * 2,  # Get more results for combination
            with_payload=True
        )

        # Perform full-text search if available (using Qdrant's sparse vectors or other methods)
        keyword_results = []
        try:
            # Use Qdrant's full-text search capability if available
            keyword_results = self.client.search(
                collection_name=self.collection_name,
                query_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content",
                            match=models.MatchText(text=query_text)
                        )
                    ]
                ),
                limit=limit * 2,
                with_payload=True
            )
        except Exception:
            # If full-text search is not available, use keyword matching in post-processing
            keyword_results = self._keyword_search_fallback(query_text, limit * 2)

        # Combine and re-rank results
        combined_results = self._combine_search_results(semantic_results, keyword_results)

        # Sort by combined score and return top results
        combined_results.sort(key=lambda x: x.score, reverse=True)
        return combined_results[:limit]

    def _keyword_search_fallback(self, query_text: str, limit: int) -> List:
        """Fallback keyword search when full-text search is not available"""
        # Perform a general search and filter results based on keyword presence
        all_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_text,  # This will use text-based search if available
            limit=limit * 4,
            with_payload=True
        )

        # Score results based on keyword presence
        keyword_lower = query_text.lower()
        scored_results = []
        for result in all_results:
            content_lower = result.payload.get("content", "").lower()
            # Count keyword matches
            keyword_matches = content_lower.count(keyword_lower)

            # Adjust score based on keyword matches
            adjusted_score = result.score + (keyword_matches * 0.1)
            result.score = adjusted_score
            scored_results.append(result)

        # Sort by adjusted score
        scored_results.sort(key=lambda x: x.score, reverse=True)
        return scored_results[:limit]

    def _combine_search_results(self, semantic_results: List, keyword_results: List) -> List:
        """Combine semantic and keyword search results"""
        # Create a map of result ID to result object for deduplication
        combined_map = {}

        # Add semantic results with weights
        for result in semantic_results:
            result_id = result.id
            # Normalize score and apply semantic weight (0.7)
            normalized_score = result.score * 0.7
            combined_map[result_id] = result
            combined_map[result_id].score = normalized_score

        # Add keyword results with weights
        for result in keyword_results:
            result_id = result.id
            if result_id in combined_map:
                # Combine scores if result already exists
                combined_map[result_id].score += result.score * 0.3
            else:
                # Apply keyword weight (0.3)
                result.score = result.score * 0.3
                combined_map[result_id] = result

        # Return combined results as a list
        return list(combined_map.values())

    def _re_rank_results(self, results: List[Dict[str, Any]], query_text: str) -> List[Dict[str, Any]]:
        """Re-rank results based on relevance to the query text"""
        # Simple re-ranking based on keyword presence and position in content
        def calculate_relevance_score(result: Dict[str, Any]) -> float:
            content = result["content"].lower()
            query = query_text.lower()

            # Count exact matches
            exact_matches = content.count(query)

            # Count word-level matches
            query_words = query.split()
            word_matches = sum(1 for word in query_words if word in content)

            # Position bonus (higher score if terms appear early in content)
            pos = content.find(query)
            pos_bonus = 0 if pos == -1 else max(0, (100 - pos) / 100.0)

            # Calculate composite score
            score = result["score"] + (exact_matches * 0.2) + (word_matches * 0.1) + pos_bonus
            return score

        # Sort results by relevance score
        results.sort(key=lambda x: calculate_relevance_score(x), reverse=True)
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