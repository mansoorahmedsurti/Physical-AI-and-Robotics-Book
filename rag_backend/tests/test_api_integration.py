# Basic integration test file for API endpoints
import pytest
from fastapi.testclient import TestClient
from rag_backend.main import create_app

@pytest.fixture
def client():
    app = create_app()
    return TestClient(app)

def test_health_endpoint(client):
    response = client.get("/health")
    assert response.status_code == 200
    assert "status" in response.json()

def test_readiness_endpoint(client):
    response = client.get("/ready")
    assert response.status_code == 200
    assert "status" in response.json()
    assert "services" in response.json()