import time
from collections import defaultdict, deque
from typing import Dict
import threading
from fastapi import HTTPException, status
try:
    # Attempt relative imports first (when run as module)
    from .utils import logger
except ImportError:
    # Fall back to absolute imports (when run as script)
    from utils import logger


class RateLimiter:
    def __init__(self):
        # Store request times for each IP address
        self.requests: Dict[str, deque] = defaultdict(deque)
        self.lock = threading.Lock()

    def is_allowed(self, identifier: str, max_requests: int, window_seconds: int) -> bool:
        """
        Check if the request from the given identifier is allowed based on rate limits

        Args:
            identifier: Unique identifier (e.g., IP address)
            max_requests: Maximum number of requests allowed
            window_seconds: Time window in seconds

        Returns:
            bool: True if request is allowed, False otherwise
        """
        with self.lock:
            now = time.time()

            # Clean up old requests outside the window
            while self.requests[identifier] and self.requests[identifier][0] <= now - window_seconds:
                self.requests[identifier].popleft()

            # Check if we're under the limit
            if len(self.requests[identifier]) < max_requests:
                self.requests[identifier].append(now)
                return True
            else:
                return False


# Global rate limiter instance
rate_limiter = RateLimiter()


def check_rate_limit(identifier: str, max_requests: int = 100, window_seconds: int = 3600):
    """
    Check if the request is within rate limits

    Args:
        identifier: Unique identifier (e.g., IP address)
        max_requests: Maximum number of requests allowed (default: 100 per hour)
        window_seconds: Time window in seconds (default: 3600 seconds = 1 hour)

    Raises:
        HTTPException: If rate limit is exceeded
    """
    if not rate_limiter.is_allowed(identifier, max_requests, window_seconds):
        logger.warning(f"Rate limit exceeded for {identifier}")
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=f"Rate limit exceeded. Maximum {max_requests} requests per {window_seconds} seconds."
        )