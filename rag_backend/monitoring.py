from datetime import datetime
from enum import Enum
import time
import json
from typing import Dict, Any, Optional
from dataclasses import dataclass
from .utils import logger
import psutil
import os


class MetricType(Enum):
    REQUEST_COUNT = "request_count"
    REQUEST_DURATION = "request_duration"
    ERROR_COUNT = "error_count"
    DOCUMENT_PROCESSED = "document_processed"
    EMBEDDING_GENERATED = "embedding_generated"


@dataclass
class Metric:
    name: str
    value: float
    labels: Dict[str, str]
    timestamp: datetime
    metric_type: MetricType


class MetricsCollector:
    def __init__(self):
        self.metrics = []
        self.request_count = 0
        self.error_count = 0
        self.document_processed = 0
        self.embedding_generated = 0

    def record_metric(self, name: str, value: float, labels: Dict[str, str] = None, metric_type: MetricType = None):
        """Record a metric"""
        if labels is None:
            labels = {}

        metric = Metric(
            name=name,
            value=value,
            labels=labels,
            timestamp=datetime.utcnow(),
            metric_type=metric_type
        )

        self.metrics.append(metric)

        # Update counters for common metrics
        if name == "request_count":
            self.request_count += 1
        elif name == "error_count":
            self.error_count += 1
        elif name == "document_processed":
            self.document_processed += 1
        elif name == "embedding_generated":
            self.embedding_generated += 1

    def get_system_metrics(self) -> Dict[str, float]:
        """Get system-level metrics"""
        return {
            "cpu_percent": psutil.cpu_percent(interval=1),
            "memory_percent": psutil.virtual_memory().percent,
            "disk_usage_percent": psutil.disk_usage('/').percent if os.name != 'nt' else psutil.disk_usage('C:\\').percent,
            "timestamp": time.time()
        }

    def get_app_metrics(self) -> Dict[str, Any]:
        """Get application-level metrics"""
        return {
            "request_count": self.request_count,
            "error_count": self.error_count,
            "document_processed": self.document_processed,
            "embedding_generated": self.embedding_generated,
            "active_sessions": 0,  # Placeholder - would track active user sessions
            "uptime_seconds": time.time(),  # Placeholder - would track actual uptime
        }

    def get_all_metrics(self) -> Dict[str, Any]:
        """Get all metrics (system + app)"""
        all_metrics = {
            "system": self.get_system_metrics(),
            "application": self.get_app_metrics(),
            "timestamp": datetime.utcnow().isoformat()
        }
        return all_metrics

    def export_metrics(self) -> str:
        """Export metrics as JSON string"""
        return json.dumps(self.get_all_metrics(), indent=2, default=str)


# Global metrics collector instance
metrics_collector = MetricsCollector()


def record_request_duration(endpoint: str, duration: float, status_code: int = 200):
    """Record request duration metric"""
    labels = {
        "endpoint": endpoint,
        "status_code": str(status_code)
    }
    metrics_collector.record_metric(
        name="request_duration_seconds",
        value=duration,
        labels=labels,
        metric_type=MetricType.REQUEST_DURATION
    )


def record_error(error_type: str, endpoint: str = ""):
    """Record error metric"""
    labels = {
        "error_type": error_type,
        "endpoint": endpoint
    }
    metrics_collector.record_metric(
        name="error_count",
        value=1,
        labels=labels,
        metric_type=MetricType.ERROR_COUNT
    )


def record_document_processed(filename: str, size_bytes: int):
    """Record document processed metric"""
    labels = {
        "filename": filename,
        "size_category": "small" if size_bytes < 100000 else "medium" if size_bytes < 1000000 else "large"
    }
    metrics_collector.record_metric(
        name="document_processed_total",
        value=1,
        labels=labels,
        metric_type=MetricType.DOCUMENT_PROCESSED
    )


def record_embedding_generated(count: int, model: str = "unknown"):
    """Record embeddings generated metric"""
    labels = {
        "model": model
    }
    metrics_collector.record_metric(
        name="embedding_generated_total",
        value=count,
        labels=labels,
        metric_type=MetricType.EMBEDDING_GENERATED
    )