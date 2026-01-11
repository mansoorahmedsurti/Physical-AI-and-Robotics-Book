# Implementation Plan: Integrated RAG Chatbot Development

**Branch**: `main` | **Date**: 2026-01-08 | **Spec**: specs/main/spec.md
**Input**: Feature specification from `/specs/main/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build and embed a Retrieval-Augmented Generation (RAG) chatbot into the published book that can answer questions using full-book context or user-selected text only. The implementation will use FastAPI backend (rag_backend/, port 8000) with OpenAI GPT-4 integration, Qdrant Cloud for vector storage, Neon Serverless Postgres for metadata, and a React chat widget integrated with the book app (port 3000).

## Technical Context

**Language/Version**: Python 3.11, JavaScript/ES6 for frontend
**Primary Dependencies**: FastAPI, uvicorn, qdrant-client, openai, psycopg2-binary, python-dotenv
**Storage**: Qdrant Cloud (vector storage), Neon Serverless Postgres (metadata, sessions, history)
**Testing**: pytest for backend, Jest for frontend
**Target Platform**: Linux server (backend), Cross-platform web browser (frontend)
**Project Type**: Web application (backend + frontend integration)
**Performance Goals**: Response time under 3 seconds for typical queries, support for 100 concurrent users, 99.9% uptime
**Constraints**: <200ms p95 for internal API calls, maintain separation from existing book application, secure API key management
**Scale/Scope**: Support for book-sized document collections, 100 concurrent users, 99.9% uptime for RAG service

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- Adheres to Accuracy through primary source verification: Using proven technologies (FastAPI, OpenAI, Qdrant, Postgres) and implementing response verification mechanisms to ensure RAG-generated content accuracy
- Follows Clarity for technical audience: Well-documented APIs and clear architecture
- Maintains Reproducibility: Using standard libraries, documented deployment procedures, and including performance benchmarking in tests
- Demonstrates Rigor: Leveraging industry-standard tools and best practices, with load testing and performance monitoring
- Addresses Ethical AI considerations: Secure handling of data, API keys, and implementing rate limiting to prevent abuse
- Ensures Comprehensiveness: Full-stack solution covering backend, storage, and frontend
- Promotes Engagement: Interactive chat interface for book content

## Project Structure

### Documentation (this feature)

```text
specs/main/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
rag_backend/              # FastAPI backend (port 8000)
├── main.py              # FastAPI application entry point
├── api/
│   ├── documents.py     # Document ingestion endpoints
│   ├── chat.py          # Chat functionality endpoints
│   └── health.py        # Health check endpoints
├── models/
│   ├── document.py      # Document data models
│   ├── chat.py          # Chat data models
│   └── database.py      # Database models
├── services/
│   ├── document_processor.py  # PDF processing and chunking
│   ├── embedding_service.py   # Embedding generation and management
│   ├── qdrant_service.py      # Vector database operations
│   └── chat_service.py        # RAG chat logic
├── database.py          # Database connection and setup
├── config.py            # Configuration and settings
├── requirements.txt     # Python dependencies
├── .env.example         # Environment variables example
└── tests/               # Backend tests
    ├── unit/
    ├── integration/
    └── conftest.py

src/                     # Existing book application (port 3000)
├── components/
│   └── RagChatWidget/   # React chat widget component
└── ...                  # Other existing files

history/prompts/         # Prompt History Records
└── main/                # RAG chatbot related prompts
    ├── [ID]-*.plan.prompt.md
    └── ...
```

**Structure Decision**: Web application structure with separate backend service (rag_backend/) for the RAG functionality while maintaining integration with the existing book application (src/). This allows the RAG service to run independently on port 8000 while the book app runs on port 3000, with a React widget embedded in the book app for seamless user experience.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple storage systems | Need both vector (Qdrant) and relational (Postgres) databases | Single database insufficient for efficient similarity search |
| Multiple services | Need separation of concerns between existing app and RAG | Tight coupling would compromise maintainability and scalability |
