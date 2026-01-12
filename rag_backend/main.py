from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
import time
import logging
try:
    # Attempt relative imports first (when run as module)
    from .api import documents, chat, health, auth, monitoring, bulk_upload
    from .utils import logger
    from .rate_limit import check_rate_limit
    from .monitoring import record_request_duration, record_error
except ImportError:
    # Fall back to absolute imports (when run as script)
    from api import documents, chat, health, auth, monitoring, bulk_upload
    from utils import logger
    from rate_limit import check_rate_limit
    from monitoring import record_request_duration, record_error

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("Starting up RAG Chatbot API...")
    from .database import db
    await db.connect()
    yield
    # Shutdown
    await db.disconnect()
    print("Shutting down RAG Chatbot API...")

def create_app():
    app = FastAPI(
        title="RAG Chatbot API",
        description="API for Retrieval-Augmented Generation chatbot",
        version="1.0.0",
        lifespan=lifespan
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, replace with specific origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Add rate limiting middleware
    @app.middleware("http")
    async def rate_limit_middleware(request: Request, call_next):
        # Skip rate limiting for certain endpoints
        if request.url.path.startswith('/health'):
            response = await call_next(request)
            return response

        # Get client IP address
        client_ip = request.client.host

        # Apply rate limiting (100 requests per hour per IP)
        check_rate_limit(client_ip, max_requests=100, window_seconds=3600)

        response = await call_next(request)
        return response

    # Add request logging and metrics middleware
    @app.middleware("http")
    async def log_requests(request: Request, call_next):
        start_time = time.time()

        response = await call_next(request)

        process_time = time.time() - start_time

        # Log request
        logger.info(f"{request.method} {request.url} - {response.status_code} - {process_time:.2f}s")

        # Record metrics
        record_request_duration(str(request.url.path), process_time, response.status_code)

        if response.status_code >= 400:
            record_error(f"HTTP_{response.status_code}", str(request.url.path))

        return response

    # Add global exception handler for validation errors
    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request, exc):
        logger.error(f"Validation error: {exc}")
        return JSONResponse(
            status_code=422,
            content={"detail": exc.errors()}
        )

    # Include API routes
    app.include_router(documents.router, prefix="/api/v1", tags=["documents"])
    app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
    app.include_router(auth.router, prefix="/api/v1", tags=["auth"])
    app.include_router(monitoring.router, prefix="/api/v1", tags=["monitoring"])
    app.include_router(bulk_upload.router, prefix="/api/v1", tags=["bulk_upload"])
    app.include_router(health.router, tags=["health"])

    # Serve the frontend
    @app.get("/")
    async def read_root():
        return {"message": "RAG Chatbot API is running. Visit /docs for API documentation or the frontend for the chat interface."}

    return app

app = create_app()

if __name__ == "__main__":
    import uvicorn
    import signal
    import sys

    def signal_handler(signum, frame):
        print("\nReceived interrupt signal. Shutting down gracefully...")
        sys.exit(0)

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    uvicorn.run(app, host="0.0.0.0", port=8000)