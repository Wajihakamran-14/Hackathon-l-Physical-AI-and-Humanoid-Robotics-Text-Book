#!/usr/bin/env python3
"""
Retrieval-Augmented Agent using OpenRouter API

This agent uses OpenRouter API directly with chat.completions for question answering
and integrates with the retrieval.py module for content-based retrieval.
"""

import os
import sys
import asyncio
import argparse
from typing import List, Dict, Any
from dotenv import load_dotenv
from retrieval import QdrantConnector, ValidationConfig
from openai import AsyncOpenAI

# -------------------------------
# Load environment variables
# -------------------------------
load_dotenv()
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY", "")

# -------------------------------
# Initialize OpenRouter client
# -------------------------------
openrouter_client = AsyncOpenAI(
    api_key=OPENROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1"
)

# -------------------------------
# Retrieval result structure
# -------------------------------
class RetrievalResult:
    def __init__(self, query: str, results: List[Dict[str, Any]], total_found: int, relevant_count: int):
        self.query = query
        self.results = results
        self.total_found = total_found
        self.relevant_count = relevant_count

# -------------------------------
# Retrieval function
# -------------------------------
def retrieve_information(query: str, top_k: int = 5, threshold: float = 0.7) -> RetrievalResult:
    config = ValidationConfig()
    qdrant_connector = QdrantConnector(
        url=config.qdrant_url,
        api_key=config.qdrant_api_key,
        collection_name=config.collection_name
    )

    if not qdrant_connector.connect_with_retry():
        return RetrievalResult(
            query=query,
            results=[{"error": f"Could not connect to Qdrant collection: {config.collection_name}"}],
            total_found=0,
            relevant_count=0
        )

    try:
        similarity_scores = qdrant_connector.search_with_text(query_text=query, top_k=top_k)
        total_found = len(similarity_scores) if similarity_scores else 0
        if not similarity_scores:
            return RetrievalResult(query=query, results=[], total_found=0, relevant_count=0)

        relevant_results = [score for score in similarity_scores if score.score >= threshold]
        query_vector = qdrant_connector.text_to_vector(query)
        all_results = qdrant_connector.search(query_vector, top_k=top_k)
        content_map = {result.id: result.content for result in all_results if result.content}

        retrieved_info = []
        for score in relevant_results[:top_k]:
            text_content = content_map.get(score.chunk_id)
            if text_content:
                retrieved_info.append({
                    "chunk_id": score.chunk_id,
                    "content": text_content[:500],
                    "similarity_score": score.score,
                    "source_url": getattr(score, 'metadata', {}).get("source_url", "") if hasattr(score, 'metadata') else ""
                })

        return RetrievalResult(
            query=query,
            results=retrieved_info,
            total_found=total_found,
            relevant_count=len(relevant_results)
        )
    except Exception as e:
        return RetrievalResult(
            query=query,
            results=[{"error": f"Error during retrieval: {str(e)}"}],
            total_found=0,
            relevant_count=0
        )

# -------------------------------
# Agent class
# -------------------------------
class RetrievalAugmentedAgent:
    def __init__(self, collection_name: str = None, similarity_threshold: float = 0.7, model_name: str = "mistralai/devstral-2512:free"):
        self.config = ValidationConfig()
        if collection_name:
            self.config.collection_name = collection_name
        self.similarity_threshold = similarity_threshold
        self.model_name = model_name
        self.client = openrouter_client

    async def query(self, user_question: str) -> str:
        try:
            # Retrieve relevant information
            retrieval_result = retrieve_information(user_question, top_k=5, threshold=self.similarity_threshold)

            if retrieval_result.results:
                # Format retrieved content for context
                context_parts = []
                for result in retrieval_result.results:
                    if 'content' in result:
                        context_parts.append(result['content'])

                if context_parts:
                    context = "\n\n".join(context_parts[:3])  # Use top 3 results
                    # Create a prompt that incorporates the retrieved context
                    prompt = f"""You are a helpful assistant that answers questions using information retrieved from a knowledge base.

Retrieved information:
{context}

Question: {user_question}

Please provide a helpful answer based on the retrieved information. If the retrieved information is not sufficient to answer the question, say so."""
                else:
                    prompt = f"Question: {user_question}\n\nI found relevant chunks but couldn't extract their content. Please answer based on your general knowledge if possible."
            else:
                prompt = f"Question: {user_question}\n\nI couldn't find any relevant information in the knowledge base to answer this question. Please answer based on your general knowledge if possible."

            # Use OpenRouter API with chat.completions
            response = await self.client.chat.completions.create(
                model=self.model_name,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.7,
                max_tokens=500
            )

            return response.choices[0].message.content
        except Exception as e:
            print(f"Error during agent query processing: {e}")
            return self._basic_retrieve_and_answer(user_question)

    def _basic_retrieve_and_answer(self, user_question: str) -> str:
        config = ValidationConfig()
        qdrant_connector = QdrantConnector(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.collection_name
        )

        if not qdrant_connector.connect_with_retry():
            return f"Could not connect to Qdrant collection: {config.collection_name}"

        try:
            similarity_scores = qdrant_connector.search_with_text(query_text=user_question, top_k=5)
            if not similarity_scores:
                return "I couldn't find any relevant information to answer your question."

            relevant_results = [score for score in similarity_scores if score.score >= self.similarity_threshold]
            if not relevant_results:
                return "I found some information but it may not be highly relevant to your question."

            query_vector = qdrant_connector.text_to_vector(user_question)
            all_results = qdrant_connector.search(query_vector, top_k=10)
            content_map = {result.id: result.content for result in all_results if result.content}

            context_parts = []
            for score in relevant_results[:3]:
                text_content = content_map.get(score.chunk_id)
                if text_content:
                    context_parts.append(text_content)

            if not context_parts:
                return "I found relevant chunks but couldn't extract their content."

            combined_context = "\n\n".join(context_parts[:3])
            return f"Based on the retrieved information:\n\n{combined_context}\n\nAnswer to your question '{user_question}': [ANSWER GENERATED FROM CONTEXT]"
        except Exception as e:
            print(f"Error during basic query processing: {e}")
            return "Sorry, I encountered an error while processing your question."

# -------------------------------
# CLI entry point
# -------------------------------
async def main():
    parser = argparse.ArgumentParser(description="AI Agent that integrates with retrieval.py using OpenRouter API")
    parser.add_argument("--query", type=str, help="Single query to process")
    parser.add_argument("--collection", type=str, default=os.getenv("QDRANT_COLLECTION", "documents"), help="Qdrant collection name")
    parser.add_argument("--threshold", type=float, default=0.7, help="Similarity threshold for relevant results")
    parser.add_argument("--model", type=str, default="mistralai/devstral-2512:free", help="Model to use for completions")
    args = parser.parse_args()

    agent_instance = RetrievalAugmentedAgent(collection_name=args.collection, similarity_threshold=args.threshold, model_name=args.model)

    if args.query:
        response = await agent_instance.query(args.query)
        print(f"Question: {args.query}\nAnswer: {response}")
    else:
        print("AI Agent initialized. Type your questions (type 'quit' to exit):")
        print(f"Using collection: {args.collection}")
        print(f"Threshold: {args.threshold}")
        print(f"Model: {args.model}")
        while True:
            try:
                user_input = input("\n> ").strip()
                if user_input.lower() in ['quit', 'exit', 'q']:
                    break
                if user_input:
                    response = await agent_instance.query(user_input)
                    print(response)
            except (KeyboardInterrupt, EOFError):
                print("\nGoodbye!")
                break

if __name__ == "__main__":
    asyncio.run(main())
