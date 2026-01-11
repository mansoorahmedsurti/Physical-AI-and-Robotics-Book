from fastapi import HTTPException
from typing import Callable, Any, Optional
import asyncio
import time
import logging
from functools import wraps
from enum import Enum


class RetryStrategy(Enum):
    LINEAR = "linear"
    EXPONENTIAL = "exponential"


class FallbackException(Exception):
    """Exception raised when all fallback options have been exhausted"""
    pass


def retry(max_attempts: int = 3, delay: float = 1.0, backoff: float = 2.0, exceptions: tuple = (Exception,)):
    """
    Decorator for retrying a function with exponential backoff
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            last_exception = None

            for attempt in range(max_attempts):
                try:
                    return await func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e
                    if attempt == max_attempts - 1:  # Last attempt
                        break

                    # Calculate delay based on backoff strategy
                    sleep_time = delay * (backoff ** attempt)
                    await asyncio.sleep(sleep_time)

            raise last_exception

        return wrapper
    return decorator


def fallback(primary_func: Callable, fallback_func: Callable, exception_types: tuple = (Exception,)):
    """
    Execute primary function, fall back to fallback function if primary fails
    """
    async def wrapper(*args, **kwargs):
        try:
            return await primary_func(*args, **kwargs)
        except exception_types:
            try:
                return await fallback_func(*args, **kwargs)
            except Exception as e:
                raise FallbackException(f"Fallback function also failed: {str(e)}")
    return wrapper


class CircuitBreakerState(Enum):
    CLOSED = "closed"
    OPEN = "open"
    HALF_OPEN = "half_open"


class CircuitBreaker:
    """
    Circuit breaker pattern implementation
    """
    def __init__(self, failure_threshold: int = 5, timeout: float = 60.0):
        self.failure_threshold = failure_threshold
        self.timeout = timeout
        self.failure_count = 0
        self.last_failure_time = None
        self.state = CircuitBreakerState.CLOSED
        self.logger = logging.getLogger(__name__)

    def __call__(self, func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            if self.state == CircuitBreakerState.OPEN:
                if time.time() - self.last_failure_time >= self.timeout:
                    self.state = CircuitBreakerState.HALF_OPEN
                    self.logger.info("Circuit breaker transitioning to HALF_OPEN")
                else:
                    raise HTTPException(status_code=503, detail="Service temporarily unavailable (circuit breaker open)")

            try:
                result = await func(*args, **kwargs)
                self.on_success()
                return result
            except Exception as e:
                self.on_failure()
                raise e

        return wrapper

    def on_success(self):
        self.failure_count = 0
        self.last_failure_time = None
        self.state = CircuitBreakerState.CLOSED

    def on_failure(self):
        self.failure_count += 1
        self.last_failure_time = time.time()

        if self.failure_count >= self.failure_threshold:
            self.state = CircuitBreakerState.OPEN
            self.logger.warning(f"Circuit breaker OPENED after {self.failure_count} failures")


def safe_execute(func: Callable, default_value: Any = None, log_errors: bool = True):
    """
    Safely execute a function, returning a default value if it fails
    """
    async def wrapper(*args, **kwargs):
        try:
            return await func(*args, **kwargs)
        except Exception as e:
            if log_errors:
                logging.error(f"Error in {func.__name__}: {str(e)}")
            return default_value
    return wrapper


class ErrorHandlingConfig:
    """
    Configuration for error handling
    """
    def __init__(self):
        self.retry_enabled = True
        self.retry_max_attempts = 3
        self.retry_delay = 1.0
        self.fallback_enabled = True
        self.circuit_breaker_enabled = True
        self.circuit_breaker_threshold = 5
        self.circuit_breaker_timeout = 60.0