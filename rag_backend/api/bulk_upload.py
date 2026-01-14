from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
try:
    # Attempt relative imports first (when run as module)
    from ..bulk_upload import bulk_uploader
    from ..auth import auth_service
    from ..models.user import UserInDB
    from ..utils import logger
except ImportError:
    # Fall back to absolute imports (when run as script)
    from bulk_upload import bulk_uploader
    from auth import auth_service
    from models.user import UserInDB
    from utils import logger

router = APIRouter()


@router.post("/bulk-upload/directory", tags=["bulk_upload"])
async def bulk_upload_directory(
    directory_path: str,
    current_user: UserInDB = Depends(auth_service.get_current_user)
):
    """
    Upload all files from a local directory to the RAG system
    """
    try:
        # Validate the directory path to prevent directory traversal
        import os
        directory_path = os.path.abspath(directory_path)

        # Basic security check - ensure the path is within allowed directories
        # For now, we'll allow relative paths and paths in common document locations
        allowed_paths = [
            os.path.abspath('.'),
            os.path.abspath('..'),
            os.path.abspath(os.path.expanduser('~/Documents')),
            os.path.abspath(os.path.expanduser('~/Desktop'))
        ]

        is_allowed = any(directory_path.startswith(allowed_path) for allowed_path in allowed_paths)

        if not is_allowed:
            raise HTTPException(status_code=400, detail="Directory path not allowed for security reasons")

        results = await bulk_uploader.upload_directory(directory_path)

        return {
            "message": f"Completed bulk upload from {directory_path}",
            "results": results
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error during bulk upload: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error during bulk upload: {str(e)}")


@router.post("/bulk-upload/file", tags=["bulk_upload"])
async def upload_single_file(
    file_path: str,
    current_user: UserInDB = Depends(auth_service.get_current_user)
):
    """
    Upload a single file to the RAG system
    """
    try:
        # Validate the file path to prevent directory traversal
        import os
        file_path = os.path.abspath(file_path)

        # Basic security check
        allowed_paths = [
            os.path.abspath('.'),
            os.path.abspath('..'),
            os.path.abspath(os.path.expanduser('~/Documents')),
            os.path.abspath(os.path.expanduser('~/Desktop'))
        ]

        is_allowed = any(file_path.startswith(allowed_path) for allowed_path in allowed_paths)

        if not is_allowed:
            raise HTTPException(status_code=400, detail="File path not allowed for security reasons")

        result = await bulk_uploader.upload_single_file(file_path)

        return result

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error uploading file {file_path}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error uploading file: {str(e)}")