#!/usr/bin/env python3
"""
FastAPI server for RAG Chatbot Integration

This server exposes the RAG agent through a FastAPI endpoint,
allowing frontend applications to send chat queries and receive responses
with retrieval context. The system extends the Docusaurus UI with a chat interface
that communicates with a FastAPI endpoint, which in turn invokes the existing
RAG agent logic from agent.py to generate responses with retrieval context.
"""

import asyncio
import os
import logging
from datetime import datetime
from typing import Dict, Any, List, Optional
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field, validator
from dotenv import load_dotenv
from fastapi.middleware.cors import CORSMiddleware

# Load environment variables
load_dotenv()

# Import the RAG agent from agent.py
from agent import RetrievalAugmentedAgent

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Retrieval-Augmented Generation Chatbot integrated with Docusaurus",
    version="1.0.0"
)

# Add CORS middleware to allow requests from the Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In development, allow all origins. In production, specify your domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------------------
# Pydantic Models for structured data (T006, T008-T011, T012-T014)
# -------------------------------

class ChatMessage(BaseModel):
    """Structure for individual chat messages exchanged between UI and backend"""
    id: str = Field(..., description="Unique identifier for the message")
    role: str = Field(..., description="Role of the message sender ('user' or 'assistant')")
    content: str = Field(..., description="The actual content of the message")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the message was created")
    sources: List[str] = Field(default=[], description="List of sources referenced in the response")

    @validator('role')
    def validate_role(cls, v):
        if v not in ['user', 'assistant']:
            raise ValueError('role must be either "user" or "assistant"')
        return v

    @validator('content')
    def validate_content(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('content cannot be empty')
        return v

class ChatRequest(BaseModel):
    """Structure for chat requests sent from frontend to backend"""
    query: str = Field(..., description="The user's query/question")
    conversation_id: Optional[str] = Field(None, description="ID to maintain conversation context")
    context: Optional[Dict] = Field(None, description="Additional context for the query")

    @validator('query')
    def validate_query(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('query cannot be empty')
        if len(v) > 2000:
            raise ValueError('query length must be between 1 and 2000 characters')
        return v

class ChatResponse(BaseModel):
    """Structure for responses from the backend to the frontend"""
    response: str = Field(..., description="The agent's response to the query")
    sources: List[Dict[str, Any]] = Field(..., description="Sources used to generate the response")
    query: str = Field(..., description="Echo of the original query")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the response was generated")
    conversation_id: str = Field(..., description="ID to maintain conversation context")
    retrieval_results: List[Dict[str, Any]] = Field(..., description="Detailed retrieval results used to generate the response")

    @validator('response')
    def validate_response(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('response cannot be empty')
        return v

    @validator('query')
    def validate_original_query(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('query cannot be empty')
        return v

class ChatHistory(BaseModel):
    """Structure for maintaining conversation history"""
    conversation_id: str = Field(..., description="Unique identifier for the conversation")
    messages: List[ChatMessage] = Field(..., description="List of messages in chronological order")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When the conversation was started")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="When the conversation was last updated")

# Define response model for health endpoint
class HealthResponse(BaseModel):
    status: str = Field(..., description="Health status of the service")
    service: str = Field(..., description="Service name")

class Error(BaseModel):
    detail: str = Field(..., description="Error message")

# -------------------------------
# API Endpoints
# -------------------------------

# Chat endpoint (T015-T025)
@app.post("/chat", response_model=ChatResponse)
async def process_chat(query_request: ChatRequest) -> ChatResponse:
    """
    Process a chat query through the RAG agent and return the response with retrieval context.

    Args:
        query_request: ChatRequest containing the query and optional parameters

    Returns:
        ChatResponse containing the query and agent's response with retrieval context
    """
    try:
        logger.info(f"Processing chat query: {query_request.query[:50]}...")

        # Create an instance of the RAG agent
        agent = RetrievalAugmentedAgent(
            collection_name=os.getenv("QDRANT_COLLECTION", "documents"),
            similarity_threshold=0.7
        )

        # Process the query using the agent
        response = await agent.query(query_request.query)

        # For now, we'll create a simple conversation ID if none provided
        conversation_id = query_request.conversation_id or f"conv_{datetime.utcnow().isoformat()}"

        # # Create a simple response structure with sources
        # # In a real implementation, the agent would return detailed sources
        sources = [{"content": "Sample source", "url": "https://example.com"}]  # Placeholder
        retrieval_results = [{"chunk_id": "sample_id", "content": "Sample content", "similarity_score": 0.8}]  # Placeholder

        # Return the response
        result = ChatResponse(
            response=response,
            sources=sources,
            query=query_request.query,
            timestamp=datetime.utcnow(),
            conversation_id=conversation_id,
            retrieval_results=retrieval_results
        )

        logger.info(f"Successfully processed chat query for conversation: {conversation_id}")
        return result

    except Exception as e:
        logger.error(f"Error processing chat query: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

# Health check endpoint (T029-T030)
@app.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """
    Health check endpoint to verify the API is running.

    Returns:
        HealthResponse with status and service information
    """
    return HealthResponse(status="healthy", service="RAG Chatbot API")

# Root endpoint (T031)
@app.get("/", response_model=Dict[str, Any])
async def root() -> Dict[str, Any]:
    """
    Root endpoint with API information and available endpoints.

    Returns:
        API information and available endpoints
    """
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": {
            "POST /chat": "Process a user query through the RAG agent with retrieval context",
            "GET /health": "Health check endpoint",
            "GET /docs": "API documentation"
        }
    }

if __name__ == "__main__":
    import uvicorn

    # Run the server
    uvicorn.run(
        "api:app",
        host="127.0.0.1",
        port=8000,
        reload=True,
        log_level="info"
    )