from fastapi import APIRouter, HTTPException, Depends, status
from fastapi.security import HTTPBearer
from datetime import timedelta
try:
    # Attempt relative imports first (when run as module)
    from ..models.user import UserCreate, Token
    from ..auth import auth_service, ACCESS_TOKEN_EXPIRE_MINUTES
except ImportError:
    # Fall back to absolute imports (when run as script)
    from models.user import UserCreate, Token
    from auth import auth_service, ACCESS_TOKEN_EXPIRE_MINUTES

router = APIRouter()
security = HTTPBearer()


@router.post("/auth/register", tags=["auth"])
async def register(user_data: UserCreate):
    """
    Register a new user
    """
    try:
        user = await auth_service.register_user(user_data)
        return {
            "id": user.id,
            "username": user.username,
            "email": user.email,
            "created_at": user.created_at
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )


@router.post("/auth/token", response_model=Token, tags=["auth"])
async def login_for_access_token(user_data: UserCreate):
    """
    OAuth2 compatible token login, get an access token for future requests
    """
    user = await auth_service.authenticate_user(user_data.username, user_data.password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = auth_service.create_access_token(
        data={"sub": user.username}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}


@router.get("/auth/me", tags=["auth"])
async def read_users_me(current_user = Depends(auth_service.get_current_user)):
    """
    Get current user information
    """
    return {
        "id": current_user.id,
        "username": current_user.username,
        "email": current_user.email,
        "created_at": current_user.created_at
    }