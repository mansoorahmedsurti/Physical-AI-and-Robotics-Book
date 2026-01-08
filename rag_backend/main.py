from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
import time
import logging
from .api import documents, chat, health
from .utils import logger

def create_app():
    app = FastAPI(
        title="RAG Chatbot API",
        description="API for Retrieval-Augmented Generation chatbot",
        version="1.0.0"
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, replace with specific origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Add request logging middleware
    @app.middleware("http")
    async def log_requests(request: Request, call_next):
        start_time = time.time()

        response = await call_next(request)

        process_time = time.time() - start_time
        logger.info(f"{request.method} {request.url} - {response.status_code} - {process_time:.2f}s")

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
    app.include_router(health.router, tags=["health"])

    return app

app = create_app()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)