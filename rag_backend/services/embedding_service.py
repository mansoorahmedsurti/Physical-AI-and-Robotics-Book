import openai
from typing import List
from ..config import settings
from ..utils import logger, EmbeddingGenerationException

class EmbeddingService:
    def __init__(self):
        openai.api_key = settings.OPENAI_API_KEY

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using OpenAI
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
            logger.error(f"Error generating embeddings: {str(e)}")
            raise EmbeddingGenerationException(f"Could not generate embeddings: {str(e)}")

    async def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for a text
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
            logger.error(f"Error generating single embedding: {str(e)}")
            raise EmbeddingGenerationException(f"Could not generate embedding: {str(e)}")

# Global embedding service instance
embedding_service = EmbeddingService()