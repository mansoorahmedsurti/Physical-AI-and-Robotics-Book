import asyncpg
import json
from typing import Dict, Any, List, Optional
from uuid import UUID
from config import settings
from utils import logger

class Database:
    def __init__(self):
        self.pool = None

    async def connect(self):
        """Establish connection to the database"""
        try:
            self.pool = await asyncpg.create_pool(
                dsn=settings.DATABASE_URL,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            print("Connected to Neon Postgres successfully")

            # Initialize tables
            await self._initialize_tables()
        except Exception as e:
            logger.error(f"Failed to connect to database: {e}")
            raise

    async def disconnect(self):
        """Close the database connection"""
        if self.pool:
            await self.pool.close()

    async def _initialize_tables(self):
        """Create required tables if they don't exist"""
        try:
            async with self.pool.acquire() as conn:
                # Create users table
                await conn.execute("""
                    CREATE TABLE IF NOT EXISTS users (
                        id SERIAL PRIMARY KEY,
                        username VARCHAR(50) UNIQUE NOT NULL,
                        email VARCHAR(255) UNIQUE,
                        hashed_password VARCHAR(255) NOT NULL,
                        created_at TIMESTAMP DEFAULT NOW(),
                        is_active BOOLEAN DEFAULT TRUE
                    )
                """)

                # Create documents table
                await conn.execute("""
                    CREATE TABLE IF NOT EXISTS documents (
                        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                        filename VARCHAR(255) NOT NULL,
                        original_name VARCHAR(255) NOT NULL,
                        content_type VARCHAR(50),
                        size_bytes INTEGER,
                        pages_count INTEGER,
                        checksum VARCHAR(64),
                        uploaded_at TIMESTAMP DEFAULT NOW(),
                        processed_at TIMESTAMP,
                        status VARCHAR(20) DEFAULT 'pending' CHECK (status IN ('pending', 'processing', 'completed', 'failed')),
                        metadata JSONB
                    )
                """)

                # Create document_chunks table
                await conn.execute("""
                    CREATE TABLE IF NOT EXISTS document_chunks (
                        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                        document_id UUID REFERENCES documents(id) ON DELETE CASCADE,
                        chunk_index INTEGER NOT NULL,
                        content TEXT NOT NULL,
                        token_count INTEGER,
                        embedding_id VARCHAR(255),
                        created_at TIMESTAMP DEFAULT NOW()
                    )
                """)

                # Create conversations table
                await conn.execute("""
                    CREATE TABLE IF NOT EXISTS conversations (
                        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                        user_id INTEGER REFERENCES users(id) ON DELETE SET NULL,
                        title VARCHAR(255),
                        created_at TIMESTAMP DEFAULT NOW(),
                        updated_at TIMESTAMP DEFAULT NOW()
                    )
                """)

                # Create messages table
                await conn.execute("""
                    CREATE TABLE IF NOT EXISTS messages (
                        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                        conversation_id UUID REFERENCES conversations(id) ON DELETE CASCADE,
                        role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
                        content TEXT NOT NULL,
                        sources JSONB,
                        timestamp TIMESTAMP DEFAULT NOW()
                    )
                """)

            logger.info("Database tables initialized successfully")
        except Exception as e:
            logger.error(f"Error initializing database tables: {e}")
            raise

    async def execute_query(self, query: str, *args):
        """Execute a query with the given arguments"""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            return await conn.fetch(query, *args)

    async def execute_command(self, command: str, *args):
        """Execute a command (INSERT, UPDATE, DELETE) with the given arguments"""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            return await conn.execute(command, *args)

    # Document-related methods
    async def create_document(self, document_data: Dict[str, Any]) -> str:
        """Create a new document record"""
        try:
            async with self.pool.acquire() as conn:
                result = await conn.fetchrow("""
                    INSERT INTO documents (
                        filename, original_name, content_type, size_bytes,
                        pages_count, checksum, metadata, status
                    ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
                    RETURNING id
                """,
                document_data.get('filename'),
                document_data.get('original_name'),
                document_data.get('content_type'),
                document_data.get('size_bytes'),
                document_data.get('pages_count'),
                document_data.get('checksum'),
                json.dumps(document_data.get('metadata')) if document_data.get('metadata') else None,
                document_data.get('status', 'pending')
                )

                return str(result['id'])
        except Exception as e:
            logger.error(f"Error creating document: {e}")
            raise

    async def get_document(self, document_id: str) -> Optional[Dict[str, Any]]:
        """Get a document by ID"""
        try:
            async with self.pool.acquire() as conn:
                result = await conn.fetchrow("""
                    SELECT * FROM documents WHERE id = $1
                """, document_id)

                if result:
                    result_dict = dict(result)
                    # Deserialize the metadata field from JSON
                    if result_dict.get('metadata') is not None:
                        import json
                        result_dict['metadata'] = json.loads(result_dict['metadata'])
                    return result_dict
                return None
        except Exception as e:
            logger.error(f"Error getting document: {e}")
            raise

    async def update_document_status(self, document_id: str, status: str):
        """Update the status of a document"""
        try:
            async with self.pool.acquire() as conn:
                await conn.execute("""
                    UPDATE documents SET status = $1, processed_at = NOW() WHERE id = $2
                """, status, document_id)
        except Exception as e:
            logger.error(f"Error updating document status: {e}")
            raise

    # User-related methods
    async def create_user(self, user_data: Dict[str, Any]) -> int:
        """Create a new user"""
        try:
            async with self.pool.acquire() as conn:
                result = await conn.fetchrow("""
                    INSERT INTO users (
                        username, email, hashed_password
                    ) VALUES ($1, $2, $3)
                    RETURNING id
                """,
                user_data.get('username'),
                user_data.get('email'),
                user_data.get('hashed_password')
                )

                return result['id']
        except Exception as e:
            logger.error(f"Error creating user: {e}")
            raise

    async def get_user_by_username(self, username: str) -> Optional[Dict[str, Any]]:
        """Get a user by username"""
        try:
            async with self.pool.acquire() as conn:
                result = await conn.fetchrow("""
                    SELECT * FROM users WHERE username = $1 AND is_active = TRUE
                """, username)

                if result:
                    return dict(result)
                return None
        except Exception as e:
            logger.error(f"Error getting user by username: {e}")
            raise

    async def get_user_by_id(self, user_id: int) -> Optional[Dict[str, Any]]:
        """Get a user by ID"""
        try:
            async with self.pool.acquire() as conn:
                result = await conn.fetchrow("""
                    SELECT * FROM users WHERE id = $1 AND is_active = TRUE
                """, user_id)

                if result:
                    return dict(result)
                return None
        except Exception as e:
            logger.error(f"Error getting user by ID: {e}")
            raise

    # Document-related methods
    async def list_documents(self, limit: int = 10, offset: int = 0, status: Optional[str] = None) -> List[Dict[str, Any]]:
        """List documents with optional filtering"""
        try:
            query = "SELECT * FROM documents"
            params = []

            if status:
                query += " WHERE status = $1"
                params.append(status)

            query += " ORDER BY uploaded_at DESC LIMIT $2 OFFSET $3"
            params.extend([limit, offset])

            async with self.pool.acquire() as conn:
                results = await conn.fetch(query, *params)

                # Convert results to dict and deserialize metadata fields
                documents = []
                for row in results:
                    row_dict = dict(row)
                    # Deserialize the metadata field from JSON
                    if row_dict.get('metadata') is not None:
                        row_dict['metadata'] = json.loads(row_dict['metadata'])
                    documents.append(row_dict)

                return documents
        except Exception as e:
            logger.error(f"Error listing documents: {e}")
            raise

# Global database instance
db = Database()