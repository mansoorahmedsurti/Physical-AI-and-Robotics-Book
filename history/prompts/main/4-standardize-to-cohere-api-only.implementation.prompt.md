---
id: 4
title: "Standardize to Cohere API only"
stage: "implementation"
date: "2026-01-12"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "main"
branch: "main"
user: "Administrator"
command: "/sp.implement"
labels: ["cohere", "api", "standardization", "refactoring"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "specs/main/spec.md"
  - "specs/main/tasks.md"
  - "specs/main/plan.md"
  - "rag_backend/services/embedding_service.py"
  - "rag_backend/services/chat_service.py"
  - "rag_backend/services/document_processor.py"
  - "rag_backend/config.py"
  - "rag_backend/.env.example"
  - "rag_backend/requirements.txt"
  - "rag_backend/DEPLOYMENT.md"
  - "specs/main/quickstart.md"
  - "specs/02_rag_chatbot/spec.md"
  - "specs/02_rag_chatbot/plan.md"
  - "specs/02_rag_chatbot/research.md"
  - "specs/02_rag_chatbot/data-model.md"
tests: []
---

# Standardize the ENTIRE project (Specs + Code) to use Cohere API only

## DIRECTIVES:
1. UPDATE DOCUMENTATION (spec.md, tasks.md, plan.md):
   - SEARCH for "OpenAI", "text-embedding-ada-002", "gpt-4", or "local embeddings".
   - REPLACE them strictly with "Cohere", "embed-english-v3.0", and "command-r".
   - UPDATE requirements to explicitly ban local model loading (due to 8GB RAM constraint).

2. REFACTOR BACKEND CODE:
   - Ensure 'rag_backend/services/embedding_service.py' uses 'CohereEmbeddings' (model='embed-english-v3.0').
   - Ensure 'rag_backend/services/chat_service.py' uses 'ChatCohere' (model='command-r').
   - DELETE any fallback logic that tries to use 'SentenceTransformer' or 'HuggingFace'.

3. FIX CONFIGURATION:
   - UPDATE 'rag_backend/config.py' to read 'COHERE_API_KEY' from env.
   - CREATE or UPDATE a '.env.example' file showing the correct variable names (COHERE_API_KEY, QDRANT_URL, DATABASE_URL).

4. VERIFICATION:
   - The goal is for spec.md and the actual code to be 100% aligned

## Response Summary

Successfully standardized the entire project to use Cohere API only. Key changes made:

### Documentation Updates:
- Updated spec.md to specify Cohere embed-english-v3.0 for embeddings and command-r for chat generation
- Updated tasks.md to reflect Cohere usage instead of OpenAI
- Updated plan.md to specify Cohere model requirements and no-local-fallback constraint
- Updated quickstart.md with correct model information

### Code Updates:
- Updated embedding_service.py to use embed-english-v3.0 model instead of embed-multilingual-v3.0
- Updated chat_service.py to use command-r model instead of command-r-plus
- Enhanced document_processor.py to enforce no-local-fallback policy by removing fallback logic
- Updated requirements.txt to document no-local-model constraint
- Updated configuration files (.env.example, DEPLOYMENT.md) with Cohere-specific information

### Additional Updates:
- Updated older specification files (specs/02_rag_chatbot/) for consistency
- Updated data models to reflect correct embedding dimensions

All files now consistently use Cohere API with the specified models and enforce the no-local-fallback policy due to 8GB RAM constraint.