## Backend Configuration

The RAG chatbot requires a backend API to function properly. Follow these steps to connect the frontend to your deployed backend:

### Environment Variables

Set the following environment variable in your deployment platform (Vercel, Netlify, etc.):

- `NEXT_PUBLIC_API_URL`: The base URL of your deployed backend (e.g., `https://your-backend-app.herokuapp.com`)

### API Endpoints

The frontend expects the backend to expose these endpoints:

- `POST /api/v1/chat/` - For chat messages
- `POST /api/v1/chat/conversation/` - For creating new conversations
- `GET /api/v1/chat/conversation/{id}` - For getting conversation history
- `POST /api/v1/documents/ingest` - For document uploads

### Example Configuration

If your backend is deployed at `https://my-rag-backend.hf.space`, set:

```
NEXT_PUBLIC_API_URL=https://my-rag-backend.hf.space
```

The frontend will automatically append `/api/v1` to this URL, making requests to `https://my-rag-backend.hf.space/api/v1/chat/`, etc.