import cohere
from typing import List
import numpy as np
from sentence_transformers import SentenceTransformer
from ..config import settings
from ..utils import logger, EmbeddingGenerationException
from ..error_handling import retry, CircuitBreaker

class EmbeddingService:
    def __init__(self):
        # Initialize Cohere client
        self.co = cohere.Client(settings.COHERE_API_KEY)
        # Initialize local embedding model as fallback
        try:
            self.local_model = SentenceTransformer('all-MiniLM-L6-v2')
            logger.info("Local embedding model loaded successfully")
        except Exception as e:
            logger.warning(f"Failed to load local embedding model: {e}. Fallback will not be available.")
            self.local_model = None

    @retry(max_attempts=3, delay=1.0, backoff=2.0, exceptions=(EmbeddingGenerationException,))
    @CircuitBreaker(failure_threshold=3, timeout=60.0)
    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere cloud embeddings only (no local fallback)
        """
        try:
            # Process in smaller batches to manage memory usage
            # Cohere supports up to 96 texts per request
            batch_size = 50  # Using a conservative batch size for Cohere
            embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]

                # Generate embeddings using Cohere
                response = self.co.embed(
                    texts=batch,
                    model="embed-multilingual-v3.0",  # Using multilingual model for broader language support
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
            logger.error(f"Error generating embeddings with Cohere: {str(e)}")

            # Raise the exception directly without fallback to local embeddings
            # This enforces cloud-only embedding usage
            raise EmbeddingGenerationException(f"Cohere embedding generation failed: {str(e)}. Local fallback disabled to ensure cloud usage.")

    @retry(max_attempts=3, delay=1.0, backoff=2.0, exceptions=(EmbeddingGenerationException,))
    @CircuitBreaker(failure_threshold=3, timeout=60.0)
    async def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for a text using Cohere cloud embeddings only (no local fallback)
        """
        try:
            # Generate embedding using Cohere
            response = self.co.embed(
                texts=[text],
                model="embed-multilingual-v3.0",  # Using multilingual model for broader language support
                input_type="search_query"  # Required parameter for the v3 model
            )

            # Extract the first (and only) embedding from the response
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error generating single embedding with Cohere: {str(e)}")

            # Raise the exception directly without fallback to local embeddings
            # This enforces cloud-only embedding usage
            raise EmbeddingGenerationException(f"Cohere single embedding generation failed: {str(e)}. Local fallback disabled to ensure cloud usage.")

# Global embedding service instance
embedding_service = EmbeddingService()