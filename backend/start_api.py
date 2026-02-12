#!/usr/bin/env python3
"""
Script to start the RAG Chatbot FastAPI server
"""

import uvicorn
import logging
import os
from api import app

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def start_server():
    """Start the FastAPI server"""
    logger.info("Starting RAG Chatbot API server...")
    logger.info("Server will be available at: http://127.0.0.1:8000")
    logger.info("API documentation available at: http://127.0.0.1:8000/docs")
    print("\nEndpoints:")
    print("  POST /chat - Process a user query through the RAG agent with retrieval context")
    print("  GET  /health - Health check endpoint")
    print("  GET  / - API information")

    uvicorn.run(
        app,
        host="127.0.0.1",
        port=8000,
        reload=False,  # Set to True for development
        log_level="info"
    )

if __name__ == "__main__":
    start_server()