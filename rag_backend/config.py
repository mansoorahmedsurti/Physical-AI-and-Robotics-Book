import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings:
    # OpenAI settings
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")

    # Qdrant settings
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_HOST: str = os.getenv("QDRANT_HOST", "")

    # Database settings
    DATABASE_URL: str = os.getenv("DATABASE_URL", "")
    POSTGRES_USER: str = os.getenv("POSTGRES_USER", "")
    POSTGRES_PASSWORD: str = os.getenv("POSTGRES_PASSWORD", "")
    POSTGRES_DB: str = os.getenv("POSTGRES_DB", "")

    # Application settings
    SECRET_KEY: str = os.getenv("SECRET_KEY", "dev-secret-key")
    DEBUG: bool = os.getenv("DEBUG", "False").lower() == "true"

settings = Settings()