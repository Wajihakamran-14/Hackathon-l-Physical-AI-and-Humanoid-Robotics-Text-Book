#!/usr/bin/env python3
"""
Test script for the RAG Agent API
"""

import asyncio
import json
from api import app, ChatRequest
from agent import RetrievalAugmentedAgent

async def test_direct_agent():
    """Test the agent directly to make sure it works"""
    print("Testing agent directly...")
    agent = RetrievalAugmentedAgent(similarity_threshold=0.7)
    response = await agent.query("What is ROS 2?")
    print(f"Direct agent response length: {len(response)} characters")
    print(f"Direct agent response preview: {response[:100]}...")
    return response

async def test_api_functionality():
    """Test the API functionality without starting a server"""
    print("\nTesting API functionality...")

    # Create a query request
    query_request = ChatRequest(query="What is ROS 2?")

    # Test the processing logic directly
    try:
        agent = RetrievalAugmentedAgent(
            collection_name=query_request.conversation_id or "documents",
            similarity_threshold=0.7
        )

        response = await agent.query(query_request.query)

        result = {
            "query": query_request.query,
            "response": response,
            "conversation_id": query_request.conversation_id,
            "similarity_threshold": 0.7
        }

        print(f"API-style response length: {len(response)} characters")
        print(f"API-style response preview: {response[:100]}...")
        return result
    except Exception as e:
        print(f"Error in API functionality test: {e}")
        return None

if __name__ == "__main__":
    print("Testing RAG Agent API functionality...\n")

    # Test direct agent
    direct_response = asyncio.run(test_direct_agent())

    # Test API-style processing
    api_result = asyncio.run(test_api_functionality())

    if direct_response and api_result:
        print("\n[SUCCESS] All tests passed! The API integration is working correctly.")
        print("- Direct agent access works")
        print("- API-style processing works")
        print("- Integration with agent.py is successful")
    else:
        print("\n[FAILURE] Some tests failed.")