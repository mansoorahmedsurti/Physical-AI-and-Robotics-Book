"""
Entry point for Hugging Face Spaces
This file provides the app instance that Hugging Face Spaces expects
"""

# Import the app instance from main.py
from main import app

# This app instance will be used by Hugging Face Spaces
# The platform looks for an 'app' variable at the module level