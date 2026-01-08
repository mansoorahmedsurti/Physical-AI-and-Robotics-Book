import asyncpg
from typing import Dict, Any, List, Optional
from uuid import UUID
from .config import settings
from .utils import logger

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
                        user_id VARCHAR(255),
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
                document_data.get('metadata'),
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

                return dict(result) if result else None
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

                return [dict(row) for row in results]
        except Exception as e:
            logger.error(f"Error listing documents: {e}")
            raise

# Global database instance
db = Database()