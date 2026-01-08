from fastapi import APIRouter, HTTPException
from uuid import UUID
from ..models.chat import QueryRequest, QueryResponse, ConversationCreate
from ..services.chat_service import chat_service
from ..database import db
from ..utils import logger

router = APIRouter()

@router.post("/chat/", response_model=QueryResponse, tags=["chat"])
async def chat(query_request: QueryRequest):
    """
    Send a message and get a RAG-enhanced response
    """
    try:
        result = await chat_service.get_answer(
            query=query_request.message,
            conversation_id=query_request.conversation_id,
            temperature=query_request.temperature
        )

        response = QueryResponse(
            conversation_id=result["conversation_id"],
            response=result["response"],
            sources=result["sources"],
            token_usage=result["token_usage"]
        )

        return response
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/chat/conversation/", tags=["chat"])
async def create_conversation(conversation_create: ConversationCreate = None):
    """
    Start a new conversation
    """
    try:
        title = conversation_create.title if conversation_create else None
        conversation_id = await chat_service.create_conversation(title=title)

        return {
            "conversation_id": conversation_id,
            "title": title,
            "created_at": "now"  # In a real implementation, this would be an actual timestamp
        }
    except Exception as e:
        logger.error(f"Error creating conversation: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/chat/conversation/{conversation_id}", tags=["chat"])
async def get_conversation_history(conversation_id: str):
    """
    Get conversation history
    """
    try:
        # Validate UUID format
        try:
            UUID(conversation_id)
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid conversation ID format")

        async with db.pool.acquire() as conn:
            # Get messages for the conversation
            messages = await conn.fetch("""
                SELECT * FROM messages
                WHERE conversation_id = $1
                ORDER BY timestamp ASC
            """, conversation_id)

            # Convert to list of dicts
            message_list = [dict(msg) for msg in messages]

        return {
            "conversation_id": conversation_id,
            "messages": message_list
        }
    except Exception as e:
        logger.error(f"Error getting conversation history: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")