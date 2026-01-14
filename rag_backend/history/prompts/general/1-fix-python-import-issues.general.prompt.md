---
id: 1
title: Fix Python Import Issues in FastAPI Application
stage: general
date_iso: 2026-01-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Administrator
command: act as a Python Architect. I am getting a 'ModuleNotFoundError' when starting my FastAPI app because the import logic is inconsistent.

The Issue:
My `__init__.py` files are trying to import sibling modules (like `documents`) using implicit absolute imports, but Python treats them as missing external libraries. Additionally, other files in the `services` and `api` folders are likely using incorrect relative imports (like `..`) that break because of how the Docker container sets the PYTHONPATH. please fix the relevant import issues as well.
labels: ["import", "python", "fastapi", "module", "relative-import", "absolute-import"]
link_spec: null
link_ticket: null
link_adr: null
link_pr: null
---

# Fix Python Import Issues in FastAPI Application

## Files Modified

- `./api/__init__.py`
- `./main.py`
- `./auth.py`
- `./database.py`
- `./bulk_upload.py`
- `./rate_limit.py`
- `./monitoring.py`
- `./api/bulk_upload.py`
- `./api/health.py`
- `./api/auth.py`
- `./api/web_scraper.py`
- `./api/chat.py`
- `./api/monitoring.py`
- `./services/document_processor.py`
- `./services/qdrant_service.py`
- `./services/web_scraper.py`
- `./services/embedding_service.py`
- `./services/chat_service.py`

## Summary of Changes

Fixed inconsistent import logic in the FastAPI application that was causing ModuleNotFoundError. The issue was caused by improper handling of relative and absolute imports when the application runs in different contexts (as a module vs. as a script).

## Solution Applied

Applied a consistent import pattern across all files:

1. **Try relative imports first**: When the module is imported as part of a package
2. **Fall back to absolute imports**: When the module is run as a script directly

Each affected file now follows this pattern:
```python
try:
    # Attempt relative imports first (when run as module)
    from .some_module import Something
except ImportError:
    # Fall back to absolute imports (when run as script)
    from some_module import Something
```

## Technical Details

- Fixed imports in `__init__.py` files to use proper package structure
- Updated relative imports (using `..` and `.`) in api and services folders with fallback logic
- Ensured all modules can be imported both as part of the package and individually
- Maintained backward compatibility with Docker container execution context

## Outcome

- ModuleNotFoundError is resolved
- Application can now be started successfully in different execution contexts
- Import logic is consistent across the entire codebase
- Both relative and absolute import scenarios are handled properly

## Prompt Text

act as a Python Architect. I am getting a 'ModuleNotFoundError' when starting my FastAPI app because the import logic is inconsistent.

The Issue:
My `__init__.py` files are trying to import sibling modules (like `documents`) using implicit absolute imports, but Python treats them as missing external libraries. Additionally, other files in the `services` and `api` folders are likely using incorrect relative imports (like `..`) that break because of how the Docker container sets the PYTHONPATH. please fix the relevant import issues as well.

## Response Summary

Fixed import issues in FastAPI application by implementing consistent fallback patterns for relative and absolute imports across all modules in the api and services directories.