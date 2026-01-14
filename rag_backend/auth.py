from datetime import datetime, timedelta
from typing import Optional
import jwt
from passlib.context import CryptContext
from fastapi import HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import os
try:
    # Attempt relative imports first (when run as module)
    from .models.user import User, UserCreate, UserInDB
    from .database import db
except ImportError:
    # Fall back to absolute imports (when run as script)
    from models.user import User, UserCreate, UserInDB
    from database import db

# Security configuration
SECRET_KEY = os.getenv("SECRET_KEY", "your-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()

class AuthService:
    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        return pwd_context.verify(plain_password, hashed_password)

    @staticmethod
    def get_password_hash(password: str) -> str:
        return pwd_context.hash(password)

    @staticmethod
    async def authenticate_user(username: str, password: str) -> Optional[UserInDB]:
        user = await db.get_user_by_username(username)
        if not user:
            return None
        if not AuthService.verify_password(password, user.hashed_password):
            return None
        return user

    @staticmethod
    def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=15)
        to_encode.update({"exp": expire})
        encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
        return encoded_jwt

    @staticmethod
    async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)) -> UserInDB:
        credentials_exception = HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
        try:
            payload = jwt.decode(credentials.credentials, SECRET_KEY, algorithms=[ALGORITHM])
            username: str = payload.get("sub")
            if username is None:
                raise credentials_exception
        except jwt.PyJWTError:
            raise credentials_exception

        user = await db.get_user_by_username(username)
        if user is None:
            raise credentials_exception
        return user

    @staticmethod
    async def register_user(user_data: UserCreate) -> User:
        # Check if user already exists
        existing_user = await db.get_user_by_username(user_data.username)
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Username already registered"
            )

        # Hash the password
        hashed_password = AuthService.get_password_hash(user_data.password)

        # Create user in database
        user_id = await db.create_user({
            "username": user_data.username,
            "email": user_data.email,
            "hashed_password": hashed_password
        })

        # Return the created user (without sensitive data)
        return User(id=user_id, username=user_data.username, email=user_data.email)


# Initialize the auth service instance
auth_service = AuthService()