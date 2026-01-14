from fastapi import APIRouter, HTTPException
try:
    # Attempt relative imports first (when run as module)
    from ..monitoring import metrics_collector
    from ..utils import logger
except ImportError:
    # Fall back to absolute imports (when run as script)
    from monitoring import metrics_collector
    from utils import logger
import time

router = APIRouter()


@router.get("/metrics", tags=["monitoring"])
async def get_metrics():
    """
    Get application metrics
    """
    try:
        return metrics_collector.get_all_metrics()
    except Exception as e:
        logger.error(f"Error getting metrics: {str(e)}")
        raise HTTPException(status_code=500, detail="Error retrieving metrics")


@router.get("/healthz", tags=["monitoring"])
async def health_check():
    """
    Health check endpoint
    """
    try:
        # Check if services are responsive
        # For now, just return basic health info
        return {
            "status": "healthy",
            "timestamp": time.time(),
            "checks": {
                "database": "ok",  # Would check actual database connectivity
                "qdrant": "ok",    # Would check actual Qdrant connectivity
                "cohere": "ok"     # Would check actual Cohere connectivity
            }
        }
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=500, detail="Health check failed")