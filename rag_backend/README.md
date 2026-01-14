# RAG Chatbot Backend for Hugging Face Spaces

This is a RAG (Retrieval-Augmented Generation) chatbot backend built with FastAPI. It enables users to upload documents and ask questions about them using AI-powered retrieval and generation.

## Features

- Document ingestion and processing
- Vector storage with Qdrant
- Cohere-powered embeddings and generation
- PostgreSQL database for metadata
- Rate limiting and monitoring

## Configuration

For Hugging Face Spaces, you can configure the application using environment variables:

- `COHERE_API_KEY`: Your Cohere API key for embeddings and generation
- `QDRANT_HOST`: Qdrant instance host (optional for demo mode)
- `QDRANT_API_KEY`: Qdrant API key (optional for demo mode)
- `DATABASE_URL`: PostgreSQL database URL (optional for demo mode)
- `SECRET_KEY`: Secret key for authentication
- `DEBUG`: Enable/disable debug mode (default: False)

## Notes

This application is designed to work with external services (Cohere, Qdrant, PostgreSQL). When deployed to Hugging Face Spaces, you may need to configure these services separately or use demo/mocked functionality for initial testing.

The application will attempt to use local embedding models as fallback when cloud services are unavailable.