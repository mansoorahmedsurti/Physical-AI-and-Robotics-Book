import cohere
from typing import List
from ..config import settings
from ..utils import logger, EmbeddingGenerationException
from ..error_handling import retry, CircuitBreaker

class EmbeddingService:
    def __init__(self):
        # Initialize Cohere client
        if not settings.COHERE_API_KEY:
            print("WARNING: COHERE_API_KEY not set. Using local embedding fallback.")
            self.co = None
        else:
            self.co = cohere.Client(settings.COHERE_API_KEY)

    @retry(max_attempts=3, delay=1.0, backoff=2.0, exceptions=(EmbeddingGenerationException,))
    @CircuitBreaker(failure_threshold=3, timeout=60.0)
    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere cloud embeddings with local fallback
        """
        try:
            # Process in smaller batches to manage memory usage
            # Cohere supports up to 96 texts per request
            batch_size = 50  # Using a conservative batch size for Cohere

            # Use local embeddings if Cohere is not available
            if not self.co:
                return await self._generate_local_embeddings(texts)

            embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]

                # Generate embeddings using Cohere
                response = self.co.embed(
                    texts=batch,
                    model="embed-english-v3.0",  # Using English model as specified in requirements
                    input_type="search_document"  # Required parameter for the v3 model
                )

                # Extract embeddings from response
                batch_embeddings = response.embeddings
                embeddings.extend(batch_embeddings)

                # Add small delay between batches to prevent API rate limits and allow GC
                import asyncio
                await asyncio.sleep(0.1)

                # Force garbage collection periodically
                import gc
                gc.collect()

            return embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")

            # Try local embeddings as fallback
            try:
                return await self._generate_local_embeddings(texts)
            except Exception as local_e:
                logger.error(f"Local embedding generation also failed: {str(local_e)}")
                raise EmbeddingGenerationException(f"Both Cohere and local embedding generation failed: {str(e)}, {str(local_e)}")

    async def _generate_local_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings using local model as fallback
        """
        try:
            # Import sentence transformer only when needed to avoid loading unnecessarily
            from sentence_transformers import SentenceTransformer
            import torch
            import numpy as np

            # Use a lightweight model suitable for Hugging Face Spaces
            model = SentenceTransformer('all-MiniLM-L6-v2')

            # Generate embeddings
            embeddings = model.encode(texts)

            # Convert to list of lists
            embeddings_list = [embedding.tolist() for embedding in embeddings]

            return embeddings_list
        except Exception as e:
            logger.error(f"Error generating local embeddings: {str(e)}")
            raise EmbeddingGenerationException(f"Local embedding generation failed: {str(e)}")

    @retry(max_attempts=3, delay=1.0, backoff=2.0, exceptions=(EmbeddingGenerationException,))
    @CircuitBreaker(failure_threshold=3, timeout=60.0)
    async def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for a text using Cohere cloud embeddings with local fallback
        """
        try:
            # Use local embeddings if Cohere is not available
            if not self.co:
                result = await self._generate_local_embeddings([text])
                return result[0] if result else []

            # Generate embedding using Cohere
            response = self.co.embed(
                texts=[text],
                model="embed-english-v3.0",  # Using English model as specified in requirements
                input_type="search_query"  # Required parameter for the v3 model
            )

            # Extract the first (and only) embedding from the response
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error generating single embedding: {str(e)}")

            # Try local embeddings as fallback
            try:
                result = await self._generate_local_embeddings([text])
                return result[0] if result else []
            except Exception as local_e:
                logger.error(f"Local single embedding generation also failed: {str(local_e)}")
                raise EmbeddingGenerationException(f"Both Cohere and local single embedding generation failed: {str(e)}, {str(local_e)}")

# Global embedding service instance
embedding_service = EmbeddingService()