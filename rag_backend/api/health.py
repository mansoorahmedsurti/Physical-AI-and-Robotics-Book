from fastapi import APIRouter
from datetime import datetime
import asyncio
from ..models.health import HealthResponse, ReadyResponse
from ..database import db
from ..services.qdrant_service import qdrant_service
from ..config import settings
from ..utils import logger

router = APIRouter()

@router.get("/health", response_model=HealthResponse, tags=["health"])
async def health_check():
    """
    Health check endpoint to verify service is running
    """
    try:
        timestamp = datetime.utcnow().isoformat()

        # Simple health check - service is running
        return HealthResponse(
            status="healthy",
            timestamp=timestamp
        )
    except Exception as e:
        logger.error(f"Health check error: {str(e)}")
        return HealthResponse(
            status="unhealthy",
            timestamp=datetime.utcnow().isoformat()
        )


@router.get("/ready", response_model=ReadyResponse, tags=["health"])
async def readiness_check():
    """
    Readiness check endpoint to verify all dependencies are available
    """
    try:
        services_status = {
            "database": False,
            "qdrant": False,
            "openai": False
        }

        # Check database connection
        try:
            if db.pool:
                async with db.pool.acquire() as conn:
                    await conn.fetchval("SELECT 1")  # Simple query to test connection
                services_status["database"] = True
        except Exception:
            services_status["database"] = False

        # Check Qdrant connection
        try:
            # Try to get collection info to verify connection
            qdrant_service.client.get_collection(qdrant_service.collection_name)
            services_status["qdrant"] = True
        except Exception:
            services_status["qdrant"] = False

        # Check OpenAI API key is configured (but don't make an expensive API call)
        services_status["openai"] = bool(settings.OPENAI_API_KEY)

        all_ready = all(services_status.values())

        return ReadyResponse(
            status="ready" if all_ready else "not_ready",
            services=services_status
        )
    except Exception as e:
        logger.error(f"Readiness check error: {str(e)}")
        return ReadyResponse(
            status="error",
            services={
                "database": False,
                "qdrant": False,
                "openai": False
            }
        )