import openai
from typing import List, Dict, Any
from uuid import UUID, uuid4
from datetime import datetime
from ..config import settings
from ..services.qdrant_service import qdrant_service
from ..services.embedding_service import embedding_service
from ..database import db
from ..utils import logger

class ChatService:
    def __init__(self):
        openai.api_key = settings.OPENAI_API_KEY

    async def get_answer(self, query: str, conversation_id: str = None, temperature: float = 0.7) -> Dict[str, Any]:
        """
        Generate an answer to a query using RAG approach
        """
        try:
            # Generate embedding for the query
            query_embedding = await embedding_service.generate_single_embedding(query)

            # Search for similar document chunks in Qdrant
            similar_chunks = await qdrant_service.search_similar(query_embedding, limit=5)

            # Construct context from similar chunks
            context_parts = []
            sources = []
            for chunk in similar_chunks:
                context_parts.append(chunk['content'])
                sources.append({
                    'document_id': chunk['document_id'],
                    'content_preview': chunk['content'][:100],
                    'page_number': chunk['page_number'],
                    'relevance_score': chunk['score']
                })

            # Combine context
            context = "\n\n".join(context_parts)

            # Construct the prompt for OpenAI
            prompt = f"""Answer the question based on the context provided. If the answer is not in the context, say "I don't know based on the provided documents."

Context:
{context}

Question: {query}

Answer:"""

            # Call OpenAI API to generate response
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",  # Could be configurable to use gpt-4 if preferred
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context."},
                    {"role": "user", "content": prompt}
                ],
                temperature=temperature
            )

            # Extract response and token usage
            answer = response.choices[0].message.content
            token_usage = {
                "prompt_tokens": response.usage.prompt_tokens,
                "completion_tokens": response.usage.completion_tokens,
                "total_tokens": response.usage.total_tokens
            }

            # Create or update conversation
            if conversation_id:
                conv_uuid = UUID(conversation_id)
            else:
                conv_uuid = uuid4()
                conversation_id = str(conv_uuid)

            # Save the message to database
            await self._save_message(conversation_id, "user", query, [])
            await self._save_message(conversation_id, "assistant", answer, sources)

            # Prepare response
            result = {
                "conversation_id": conversation_id,
                "response": answer,
                "sources": sources,
                "token_usage": token_usage
            }

            return result
        except Exception as e:
            logger.error(f"Error generating answer: {str(e)}")
            raise

    async def _save_message(self, conversation_id: str, role: str, content: str, sources: List[Dict[str, Any]]):
        """
        Save a message to the database
        """
        try:
            async with db.pool.acquire() as conn:
                await conn.execute("""
                    INSERT INTO messages (conversation_id, role, content, sources)
                    VALUES ($1, $2, $3, $4)
                """, conversation_id, role, content, sources)
        except Exception as e:
            logger.error(f"Error saving message to database: {str(e)}")
            raise

    async def create_conversation(self, title: str = None) -> str:
        """
        Create a new conversation
        """
        try:
            conversation_id = str(uuid4())

            async with db.pool.acquire() as conn:
                await conn.execute("""
                    INSERT INTO conversations (id, title)
                    VALUES ($1, $2)
                """, conversation_id, title or f"Conversation {datetime.now().strftime('%Y-%m-%d %H:%M')}")

            return conversation_id
        except Exception as e:
            logger.error(f"Error creating conversation: {str(e)}")
            raise

# Global chat service instance
chat_service = ChatService()