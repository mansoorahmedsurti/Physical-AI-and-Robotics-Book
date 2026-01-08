from pydantic import BaseModel
from typing import Dict, Optional

class HealthResponse(BaseModel):
    status: str
    timestamp: str

class ReadyResponse(BaseModel):
    status: str
    services: Dict[str, bool]