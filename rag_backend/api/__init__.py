try:
    # Attempt relative imports first (when run as module)
    from . import documents, chat, health, auth, monitoring, bulk_upload
except ImportError:
    # Fall back to absolute imports (when run as script)
    import documents, chat, health, auth, monitoring, bulk_upload