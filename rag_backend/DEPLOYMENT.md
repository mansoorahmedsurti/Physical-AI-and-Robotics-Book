# RAG Chatbot Deployment Guide

## Overview
This guide describes how to deploy the RAG Chatbot backend service to production environments.

## Prerequisites
- Docker and Docker Compose
- Access to cloud providers (AWS, GCP, Azure, or equivalent)
- Domain name and SSL certificate (optional but recommended)
- External services:
  - Neon Postgres database
  - Qdrant Cloud instance
  - OpenAI API key (required for cloud-based embeddings - no local fallback available)

## Environment Configuration

### Required Environment Variables
Create a `.env.production` file with the following variables:

```env
# Cohere Configuration
COHERE_API_KEY=your-cohere-api-key-here

# Qdrant Configuration
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_HOST=https://your-cluster.qdrant.tech

# Database Configuration
DATABASE_URL=postgresql://username:password@your-neon-db.neon.tech/dbname
POSTGRES_USER=username
POSTGRES_PASSWORD=secure-password
POSTGRES_DB=rag_chatbot

# Application Configuration
SECRET_KEY=a-very-long-random-secret-key
DEBUG=False
```

## Docker Deployment

### Building the Image
```bash
cd rag_backend
docker build -t rag-chatbot:latest .
```

### Running with Docker
```bash
docker run -d \
  --name rag-chatbot \
  --env-file .env.production \
  -p 8000:8000 \
  rag-chatbot:latest
```

### Running with Docker Compose
Create a `docker-compose.yml`:

```yaml
version: '3.8'

services:
  rag-backend:
    build: ./rag_backend
    ports:
      - "8000:8000"
    environment:
      - COHERE_API_KEY=${COHERE_API_KEY}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - QDRANT_HOST=${QDRANT_HOST}
      - DATABASE_URL=${DATABASE_URL}
      - SECRET_KEY=${SECRET_KEY}
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s
```

Then run:
```bash
docker-compose up -d
```

## Kubernetes Deployment (Optional)

For production environments, consider deploying with Kubernetes:

1. Create Kubernetes manifests for the service
2. Use secrets for sensitive environment variables
3. Set up ingress controller for external access
4. Configure horizontal pod autoscaling based on demand

## Health Checks and Monitoring

### Health Endpoints
- `GET /health` - Basic health check
- `GET /ready` - Readiness check (verifies dependencies)

### Monitoring
Monitor these key metrics:
- Response times
- Error rates
- API token usage
- Database connection pool
- Qdrant connectivity

## Scaling Considerations

### Vertical Scaling
- Increase instance resources (CPU/RAM) for higher throughput
- Adjust uvicorn workers based on CPU cores

### Horizontal Scaling
- Deploy behind a load balancer
- Use external Redis for session management (if needed)
- Ensure database connection pooling is optimized

## Security Best Practices

1. Use HTTPS in production
2. Rotate API keys regularly
3. Limit database access with least privilege
4. Monitor API usage for anomalies
5. Implement proper input validation
6. Regular security scanning of dependencies

## Backup and Recovery

1. Regular database backups through Neon's automated backups
2. Document recovery procedures
3. Test backup restoration periodically
4. Store embeddings backup if needed

## Troubleshooting

### Common Issues
- Database connectivity issues: Check connection string and credentials
- Qdrant connectivity: Verify API key and host
- Cohere API errors: Check quota and API key validity (Note: Cloud-based embeddings only, no local fallback available)
- High response times: Monitor token usage and scale resources

### Logs
Access application logs through:
- Docker: `docker logs rag-chatbot`
- Kubernetes: `kubectl logs deployment/rag-backend`