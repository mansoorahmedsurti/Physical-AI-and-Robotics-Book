import openai
from typing import List
import numpy as np
from sentence_transformers import SentenceTransformer
from config import settings
from utils import logger, EmbeddingGenerationException
from error_handling import retry, CircuitBreaker

class EmbeddingService:
    def __init__(self):
        openai.api_key = settings.OPENAI_API_KEY
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
        Generate embeddings for a list of texts using OpenAI with fallback to local model
        """
        try:
            # OpenAI embedding API is synchronous, so we'll handle it as such
            # In a real implementation, we might want to batch these calls
            embeddings = []

            for text in texts:
                # Truncate text if too long (OpenAI has a limit)
                truncated_text = text[:8192]  # OpenAI's limit is 8192 tokens

                response = openai.Embedding.create(
                    input=truncated_text,
                    model="text-embedding-ada-002"
                )

                embedding = response['data'][0]['embedding']
                embeddings.append(embedding)

            return embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings with OpenAI: {str(e)}")

            # Try fallback to local model if available
            if self.local_model:
                logger.info("Falling back to local embedding model")
                try:
                    # Generate embeddings using local model
                    embeddings = self.local_model.encode(texts).tolist()
                    return embeddings
                except Exception as local_e:
                    logger.error(f"Local embedding model also failed: {str(local_e)}")
            else:
                logger.warning("Local embedding model not available for fallback")

            # If both methods failed, raise the original exception
            raise EmbeddingGenerationException(f"Could not generate embeddings: {str(e)}")

    @retry(max_attempts=3, delay=1.0, backoff=2.0, exceptions=(EmbeddingGenerationException,))
    @CircuitBreaker(failure_threshold=3, timeout=60.0)
    async def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for a text with fallback to local model
        """
        try:
            # Truncate text if too long
            truncated_text = text[:8192]

            response = openai.Embedding.create(
                input=truncated_text,
                model="text-embedding-ada-002"
            )

            return response['data'][0]['embedding']
        except Exception as e:
            logger.error(f"Error generating single embedding with OpenAI: {str(e)}")

            # Try fallback to local model if available
            if self.local_model:
                logger.info("Falling back to local embedding model for single embedding")
                try:
                    # Generate embedding using local model
                    embedding = self.local_model.encode([truncated_text])[0].tolist()
                    return embedding
                except Exception as local_e:
                    logger.error(f"Local embedding model also failed for single embedding: {str(local_e)}")
            else:
                logger.warning("Local embedding model not available for fallback")

            # If both methods failed, raise the original exception
            raise EmbeddingGenerationException(f"Could not generate embedding: {str(e)}")

# Global embedding service instance
embedding_service = EmbeddingService()