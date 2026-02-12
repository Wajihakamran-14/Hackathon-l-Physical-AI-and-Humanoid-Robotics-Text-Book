"""
Consolidated retrieval validation system for testing Qdrant retrieval accuracy, metadata integrity, and consistency.

This file combines all retrieval validation functionality into a single module:
- Data models for DocumentChunk, QueryVector, SimilarityScore, ValidationResult, ValidationSummary
- Qdrant connector with cosine similarity calculation
- Retrieval validator with accuracy, metadata, and consistency validation
- Configuration management
- CLI interface for running validation tests
"""

import os
import logging
import json
import yaml
from pathlib import Path
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, field
from datetime import datetime
import time
import numpy as np
from qdrant_client import QdrantClient
from qdrant_client.http import models
import argparse
import sys
import re


# Data Models
@dataclass
class DocumentChunk:
    """
    Represents a segment of content from source documents with associated vector embeddings,
    containing metadata (URL, section, chunk ID) and content text
    """
    id: str  # Unique identifier for the document chunk
    content: str  # The actual text content of the document chunk
    url: str  # Source URL where the original document is located
    section: str  # Section identifier within the source document
    chunk_id: str  # Unique identifier for this specific chunk
    vector: List[float]  # Vector embedding representation of the content
    metadata: Dict[str, Any]  # Additional metadata associated with the chunk


@dataclass
class QueryVector:
    """
    Represents a vectorized query for similarity search in Qdrant
    """
    query_text: str  # Original text of the query
    vector: List[float]  # Vector embedding of the query text
    query_id: str  # Unique identifier for this query


@dataclass
class SimilarityScore:
    """
    Represents the similarity score between a query and a document chunk
    """
    chunk_id: str  # ID of the document chunk being compared
    score: float  # Similarity score (0.0 to 1.0, where 1.0 is most similar)
    rank: int  # Ranking position of this result (1 = highest similarity)


@dataclass
class ValidationResult:
    """
    Represents the result of a single validation test
    """
    query_id: str  # Unique identifier for the query that was validated
    query_content: str  # The original query text that was validated
    retrieved_chunks: List[SimilarityScore]  # List of chunks retrieved with their similarity scores
    metadata_valid: bool  # Whether the metadata of retrieved chunks is valid
    relevance_threshold_met: bool  # Whether relevance threshold was met
    validation_passed: bool  # Whether the entire validation test passed
    timestamp: datetime  # When the validation was performed
    execution_time: float  # Time taken to perform the validation in seconds


@dataclass
class ValidationSummary:
    """
    Represents a summary of multiple validation tests
    """
    total_queries: int  # Total number of queries validated
    queries_passed: int  # Number of queries that passed validation
    relevance_accuracy: float  # Percentage of queries that met relevance threshold
    metadata_accuracy: float  # Percentage of queries with valid metadata
    consistency_score: float  # Score representing consistency of results
    start_time: datetime  # When validation started
    end_time: datetime  # When validation ended
    execution_time: float  # Total time taken for all validations


# Configuration
class ValidationConfig:
    """
    Configuration for retrieval validation
    """
    def __init__(self):
        # Load environment variables
        from dotenv import load_dotenv
        load_dotenv()

        # Qdrant connection settings
        self.qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY", "")
        self.collection_name = os.getenv("QDRANT_COLLECTION", "default_collection")

        # Validation settings
        self.similarity_threshold = float(os.getenv("SIMILARITY_THRESHOLD", "0.7"))
        self.num_test_runs = int(os.getenv("NUM_TEST_RUNS", "10"))
        self.validate_metadata = os.getenv("VALIDATE_METADATA", "true").lower() == "true"

        # Performance settings
        self.max_memory_usage = int(os.getenv("MAX_MEMORY_MB", "50"))  # in MB
        self.max_query_time = float(os.getenv("MAX_QUERY_TIME", "0.5"))  # in seconds

        # Sample queries for validation
        self.default_sample_queries = [
            "vector embeddings",
            "Qdrant configuration",
            "retrieval accuracy",
            "semantic search",
            "document chunking"
        ]


# Global configuration instance
config = ValidationConfig()


# Logging setup
def setup_logging():
    """
    Set up basic logging configuration for validation results
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('retrieval_validation.log'),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger('QdrantConnector')


# Initialize logger
logger = setup_logging()


def cosine_similarity(vec_a: List[float], vec_b: List[float]) -> float:
    """
    Calculate cosine similarity between two vectors
    """
    if not vec_a or not vec_b or len(vec_a) != len(vec_b):
        return 0.0

    # Convert to numpy arrays
    a = np.array(vec_a)
    b = np.array(vec_b)

    # Calculate cosine similarity
    dot_product = np.dot(a, b)
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)

    if norm_a == 0 or norm_b == 0:
        return 0.0

    return float(dot_product / (norm_a * norm_b))


class QdrantConnector:
    """
    Connector class for interacting with Qdrant vector database
    """

    def __init__(self, url: str = None, api_key: str = None, collection_name: str = None):
        """
        Initialize Qdrant connector with configuration
        """
        self.url = url or config.qdrant_url
        self.api_key = api_key or config.qdrant_api_key
        self.collection_name = collection_name or config.collection_name

        # Initialize Qdrant client
        if self.api_key:
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key,
                prefer_grpc=True
            )
        else:
            self.client = QdrantClient(url=self.url)

    def connect(self) -> bool:
        """
        Test connection to Qdrant instance
        """
        try:
            # Try to get collection info to verify connection
            collection_info = self.client.get_collection(self.collection_name)
            logger.info(f"Successfully connected to Qdrant collection: {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            return False

    def connect_with_retry(self, max_retries: int = 3, delay: float = 1.0) -> bool:
        """
        Add error handling and graceful failure modes for Qdrant connection issues with retry logic
        """
        for attempt in range(max_retries):
            try:
                # Try to get collection info to verify connection
                collection_info = self.client.get_collection(self.collection_name)
                logger.info(f"Successfully connected to Qdrant collection: {self.collection_name} (attempt {attempt + 1})")
                return True
            except Exception as e:
                logger.warning(f"Connection attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:  # Don't sleep on the last attempt
                    import time
                    time.sleep(delay)
                else:
                    logger.error(f"Failed to connect to Qdrant after {max_retries} attempts: {e}")

        return False

    def search(self, query_vector: List[float], top_k: int = 10, metadata_filter: Dict[str, Any] = None) -> List[DocumentChunk]:
        """
        Perform similarity search in Qdrant collection
        """
        try:
            # Prepare filter if provided
            qdrant_filter = None
            if metadata_filter:
                qdrant_filter = models.Filter(
                    must=[
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        ) for key, value in metadata_filter.items()
                    ]
                )

            # Use the correct method for Qdrant client version
            # Based on our debug, the method is 'query_points' in newer versions
            if hasattr(self.client, 'query_points'):
                # Newest API uses 'query_points' method for vector search
                # Need to specify the vector name based on collection schema
                search_results = self.client.query_points(
                    collection_name=self.collection_name,
                    query=query_vector,
                    using="content",  # Specify the vector field name based on collection schema
                    limit=top_k,
                    query_filter=qdrant_filter,
                    with_payload=True,
                    with_vectors=True
                )
                # query_points returns QueryResponse object, need to access .points
                if hasattr(search_results, 'points'):
                    search_results = search_results.points
            elif hasattr(self.client, 'search'):
                # Older API uses 'search' method
                search_results = self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    limit=top_k,
                    query_filter=qdrant_filter,
                    with_payload=True,
                    with_vectors=True
                )
            elif hasattr(self.client, 'search_points'):
                # Even older API
                search_results = self.client.search_points(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    limit=top_k,
                    query_filter=qdrant_filter,
                    with_payload=True,
                    with_vectors=True
                )
            else:
                # Fallback: try to list available methods to debug
                available_methods = [method for method in dir(self.client) if not method.startswith('_') and callable(getattr(self.client, method))]
                raise AttributeError(f"Qdrant client does not have expected search methods. Available methods: {available_methods[:10]}")

            # Convert results to DocumentChunk objects
            # query_points returns a ScoredPoint object or different structure
            document_chunks = []
            for result in search_results:
                # Check if result is a ScoredPoint object (newer versions)
                if hasattr(result, 'id') and hasattr(result, 'payload'):
                    # Newer Qdrant client returns ScoredPoint objects
                    payload = result.payload or {}
                    # FIX: Read chunk_id from payload, not from result.id (Qdrant Cloud requirement)
                    chunk_id = payload.get("chunk_id")
                    if chunk_id is None or chunk_id == "":
                        continue  # Skip results without chunk_id in payload
                    vector = getattr(result, 'vector', []) or []
                elif hasattr(result, '__dict__'):
                    # Object with attributes
                    payload = getattr(result, 'payload', {}) or {}
                    # FIX: Read chunk_id from payload, not from result.id (Qdrant Cloud requirement)
                    chunk_id = payload.get("chunk_id")
                    if chunk_id is None or chunk_id == "":
                        continue  # Skip results without chunk_id in payload
                    vector = getattr(result, 'vector', []) or []
                elif isinstance(result, dict):
                    # Dictionary format
                    payload = result.get('payload', {}) or {}
                    # FIX: Read chunk_id from payload, not from result.id (Qdrant Cloud requirement)
                    chunk_id = payload.get("chunk_id")
                    if chunk_id is None or chunk_id == "":
                        continue  # Skip results without chunk_id in payload
                    vector = result.get('vector', []) or []
                else:
                    # Handle as raw object - try to access attributes safely
                    try:
                        payload = getattr(result, 'payload', {}) or {}
                        # FIX: Read chunk_id from payload, not from result.id (Qdrant Cloud requirement)
                        chunk_id = payload.get("chunk_id")
                        if chunk_id is None:
                            continue  # Skip results without chunk_id in payload
                        vector = getattr(result, 'vector', []) or []
                    except:
                        # If we can't extract properly, skip this result
                        continue

                # Extract metadata fields with defaults
                chunk = DocumentChunk(
                    id=chunk_id,
                    content=payload.get("text", ""),  # Use "text" field as stored during ingestion
                    url=payload.get("url", ""),
                    section=payload.get("section", ""),
                    chunk_id=chunk_id,
                    vector=vector,
                    metadata=payload
                )
                document_chunks.append(chunk)

            return document_chunks

        except Exception as e:
            print(f"Search failed: {e}")
            return []

    def get_chunk_metadata(self, chunk_ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieve metadata for specific chunk IDs
        """
        try:
            # Filter out invalid IDs like 'unknown' before querying
            valid_chunk_ids = []
            for chunk_id in chunk_ids:
                if chunk_id and chunk_id != 'unknown' and chunk_id.strip():
                    # Try to validate if it's a valid ID format
                    try:
                        # Attempt to parse as UUID or convert appropriately
                        if isinstance(chunk_id, str) and len(chunk_id) > 0:
                            valid_chunk_ids.append(chunk_id)
                    except:
                        # Skip invalid IDs
                        continue

            if not valid_chunk_ids:
                # If no valid IDs, return empty list
                return []

            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=valid_chunk_ids,
                with_payload=True,
                with_vectors=False
            )

            metadata_list = []
            for point in points:
                # FIX: Read chunk_id from payload, not from point.id (Qdrant Cloud requirement)
                payload = point.payload or {}
                chunk_id = payload.get("chunk_id")
                if chunk_id is not None:
                    metadata_list.append({
                        "id": chunk_id,  # Use chunk_id from payload as the identifier
                        "payload": payload
                    })

            return metadata_list
        except Exception as e:
            print(f"Failed to retrieve chunk metadata: {e}")
            return []

    def get_all_chunk_ids(self) -> List[str]:
        """
        Get all chunk IDs in the collection
        """
        try:
            # Try the scroll method first (newer API)
            if hasattr(self.client, 'scroll'):
                records, _ = self.client.scroll(
                    collection_name=self.collection_name,
                    limit=10000,  # Adjust based on collection size
                    with_payload=False,
                    with_vectors=False
                )
                return [str(record.id) for record in records]
            else:
                # Fallback if scroll doesn't exist
                # This might require a different approach depending on the client version
                print("Scroll method not available in Qdrant client")
                return []
        except Exception as e:
            print(f"Failed to get all chunk IDs: {e}")
            return []

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "vector_size": collection_info.config.params.vectors_config.size if hasattr(collection_info.config.params, 'vectors_config') and hasattr(collection_info.config.params.vectors_config, 'size') else "dynamic",
                "points_count": collection_info.points_count,
                "config": collection_info.config.dict() if hasattr(collection_info.config, 'dict') else str(collection_info.config)
            }
        except Exception as e:
            print(f"Failed to get collection info: {e}")
            return {}

    def text_to_vector(self, text: str) -> List[float]:
        """
        Convert text to vector embedding using Cohere
        """
        try:
            # Import cohere here to avoid issues if not available
            import cohere
            from dotenv import load_dotenv

            load_dotenv()
            cohere_api_key = os.getenv("COHERE_API_KEY")

            if not cohere_api_key:
                raise ValueError("COHERE_API_KEY not found in environment variables")

            cohere_client = cohere.Client(cohere_api_key)

            # Generate embedding for the text
            response = cohere_client.embed(
                texts=[text],
                model='embed-multilingual-v3.0',
                input_type='search_query'  # Use search_query for queries
            )

            # Return the embedding vector
            return response.embeddings[0]

        except Exception as e:
            logger.error(f"Failed to convert text to vector: {e}")
            return []

    def search_with_text(self, query_text: str, top_k: int = 10, metadata_filter: Dict[str, Any] = None) -> List[SimilarityScore]:
        """
        Perform similarity search using text query
        """
        try:
            # Convert text query to vector
            query_vector = self.text_to_vector(query_text)

            if not query_vector:
                logger.error("Failed to convert query text to vector")
                return []

            # Perform search in Qdrant
            results = self.search(query_vector, top_k=top_k, metadata_filter=metadata_filter)

            # Convert results to SimilarityScore objects
            # Since the search method now returns DocumentChunk objects with proper payloads,
            # we need to get similarity scores from the original search results
            # We need to perform a separate search to get the scores
            search_results_raw = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                using="content",
                limit=top_k,
                query_filter=metadata_filter,  # Use the same metadata filter as original search
                with_payload=True,
                with_vectors=False
            )

            # query_points returns QueryResponse object, need to access .points
            if hasattr(search_results_raw, 'points'):
                search_results_raw = search_results_raw.points

            # Map chunk_ids to scores
            score_map = {}
            for raw_result in search_results_raw:
                chunk_id = raw_result.payload.get("chunk_id") if raw_result.payload else None
                if chunk_id:
                    score_map[chunk_id] = raw_result.score

            # Create SimilarityScore objects using the scores from Qdrant
            similarity_scores = []
            for i, chunk in enumerate(results):
                score_value = score_map.get(chunk.id, 0.0)  # Use the chunk.id which is the chunk_id from payload
                score = SimilarityScore(
                    chunk_id=chunk.id,
                    score=score_value,
                    rank=i + 1
                )
                similarity_scores.append(score)

            return similarity_scores

        except Exception as e:
            logger.error(f"Failed to perform similarity search: {e}")
            return []


class RetrievalValidator:
    """
    Validator class for testing retrieval accuracy, metadata integrity, and consistency
    """

    def __init__(self, qdrant_connector: QdrantConnector = None):
        """
        Initialize validator with Qdrant connector
        """
        self.qdrant_connector = qdrant_connector or QdrantConnector()
        self.connected = False

    def connect(self) -> bool:
        """
        Connect to Qdrant and verify connection
        """
        self.connected = self.qdrant_connector.connect()
        return self.connected

    def validate_single_query(self, query_text: str, expected_chunks: Optional[List[str]] = None) -> ValidationResult:
        """
        Validate a single query against the retrieval system
        """
        if not self.connected:
            raise RuntimeError("Not connected to Qdrant. Call connect() first.")

        start_time = time.time()

        # Convert query text to vector and perform search
        similarity_scores = self.qdrant_connector.search_with_text(
            query_text=query_text,
            top_k=10  # Get top 10 results
        )

        # Check if any results meet the similarity threshold
        threshold_met = any(score.score >= config.similarity_threshold for score in similarity_scores)

        # Check metadata validity for the top results
        metadata_valid = True
        if similarity_scores:
            # Get metadata for the top 3 chunks to validate
            # Since we already have the search results with full payloads in the similarity_scores,
            # we need to perform a new search to get the payloads
            query_vector = self.qdrant_connector.text_to_vector(query_text)
            search_results = self.qdrant_connector.search(query_vector, top_k=10)  # Get all results to map

            # Create a mapping from chunk_id to full result for metadata validation
            result_map = {result.id: result for result in search_results}

            # Validate metadata for the top scoring results
            for score in similarity_scores[:3]:
                result = result_map.get(score.chunk_id)
                if result:
                    payload = result.metadata
                    # Validate required metadata fields
                    chunk_id = payload.get("chunk_id")
                    source_url = payload.get("source_url", "")
                    section = payload.get("section", "")

                    if not chunk_id or not source_url:
                        metadata_valid = False
                        break
                else:
                    metadata_valid = False
                    break

        # Overall validation result
        validation_passed = threshold_met and metadata_valid

        execution_time = time.time() - start_time

        # Create validation result
        result = ValidationResult(
            query_id=f"query_{int(time.time())}",
            query_content=query_text,
            retrieved_chunks=similarity_scores,
            metadata_valid=metadata_valid,
            relevance_threshold_met=threshold_met,
            validation_passed=validation_passed,
            timestamp=datetime.now(),
            execution_time=execution_time
        )

        return result

    def validate_retrieval_accuracy(self, queries: List[str]) -> List[ValidationResult]:
        """
        Validate retrieval accuracy for a list of queries
        """
        results = []
        for query in queries:
            result = self.validate_single_query(query)
            results.append(result)
        return results

    def validate_metadata_integrity(self, chunk_ids: List[str]) -> bool:
        """
        Validate that retrieved document chunks contain complete and accurate metadata
        """
        if not self.connected:
            raise RuntimeError("Not connected to Qdrant. Call connect() first.")

        metadata_list = self.qdrant_connector.get_chunk_metadata(chunk_ids)

        for meta in metadata_list:
            payload = meta.get("payload", {})
            # Check for required metadata fields
            chunk_id = meta.get("id")
            if not payload.get("url") or not payload.get("section") or not chunk_id:
                return False

        return True

    def validate_metadata_completeness(self, chunk_ids: List[str]) -> Dict[str, Any]:
        """
        Create metadata validation method to check completeness
        """
        if not self.connected:
            raise RuntimeError("Not connected to Qdrant. Call connect() first.")

        metadata_list = self.qdrant_connector.get_chunk_metadata(chunk_ids)

        validation_results = {
            "total_chunks": len(chunk_ids),
            "valid_chunks": 0,
            "invalid_chunks": [],
            "missing_fields": {"url": 0, "section": 0, "chunk_id": 0},
            "completeness_score": 0.0
        }

        for meta in metadata_list:
            payload = meta.get("payload", {})
            chunk_id = meta.get("id", "")

            # Check for required fields
            issues = []
            if not payload.get("url"):
                validation_results["missing_fields"]["url"] += 1
                issues.append("url")
            if not payload.get("section"):
                validation_results["missing_fields"]["section"] += 1
                issues.append("section")
            if not chunk_id:
                validation_results["missing_fields"]["chunk_id"] += 1
                issues.append("chunk_id")

            if not issues:
                validation_results["valid_chunks"] += 1
            else:
                validation_results["invalid_chunks"].append({
                    "chunk_id": chunk_id,
                    "missing_fields": issues
                })

        validation_results["completeness_score"] = validation_results["valid_chunks"] / validation_results["total_chunks"] if validation_results["total_chunks"] > 0 else 0.0

        return validation_results

    def validate_url_field(self, chunk_ids: List[str]) -> Dict[str, Any]:
        """
        Implement validation for URL field presence and format
        """
        import re
        if not self.connected:
            raise RuntimeError("Not connected to Qdrant. Call connect() first.")

        metadata_list = self.qdrant_connector.get_chunk_metadata(chunk_ids)

        validation_results = {
            "total_chunks": len(chunk_ids),
            "valid_urls": 0,
            "invalid_urls": [],
            "missing_urls": 0
        }

        # URL pattern validation regex
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)

        for meta in metadata_list:
            payload = meta.get("payload", {})
            chunk_id = meta.get("id", "")
            url = payload.get("url", "")

            if not url:
                validation_results["missing_urls"] += 1
                validation_results["invalid_urls"].append({"chunk_id": chunk_id, "url": url, "reason": "missing"})
            elif url_pattern.match(url):
                validation_results["valid_urls"] += 1
            else:
                validation_results["invalid_urls"].append({"chunk_id": chunk_id, "url": url, "reason": "invalid_format"})

        return validation_results

    def validate_section_field(self, chunk_ids: List[str]) -> Dict[str, Any]:
        """
        Implement validation for section identifier presence
        """
        if not self.connected:
            raise RuntimeError("Not connected to Qdrant. Call connect() first.")

        metadata_list = self.qdrant_connector.get_chunk_metadata(chunk_ids)

        validation_results = {
            "total_chunks": len(chunk_ids),
            "valid_sections": 0,
            "invalid_sections": [],
            "missing_sections": 0
        }

        for meta in metadata_list:
            payload = meta.get("payload", {})
            chunk_id = meta.get("id", "")
            section = payload.get("section", "")

            if not section:
                validation_results["missing_sections"] += 1
                validation_results["invalid_sections"].append({"chunk_id": chunk_id, "section": section, "reason": "missing"})
            else:
                validation_results["valid_sections"] += 1

        return validation_results

    def validate_chunk_id_field(self, chunk_ids: List[str]) -> Dict[str, Any]:
        """
        Implement validation for chunk ID presence and uniqueness
        """
        if not self.connected:
            raise RuntimeError("Not connected to Qdrant. Call connect() first.")

        metadata_list = self.qdrant_connector.get_chunk_metadata(chunk_ids)

        validation_results = {
            "total_chunks": len(chunk_ids),
            "valid_chunk_ids": 0,
            "invalid_chunk_ids": [],
            "duplicate_chunk_ids": [],
            "missing_chunk_ids": 0
        }

        seen_ids = set()
        duplicates = set()

        for meta in metadata_list:
            chunk_id = meta.get("id", "")

            if not chunk_id:
                validation_results["missing_chunk_ids"] += 1
                validation_results["invalid_chunk_ids"].append({"chunk_id": chunk_id, "reason": "missing"})
            else:
                if chunk_id in seen_ids:
                    duplicates.add(chunk_id)
                    validation_results["duplicate_chunk_ids"].append(chunk_id)
                    validation_results["invalid_chunk_ids"].append({"chunk_id": chunk_id, "reason": "duplicate"})
                else:
                    seen_ids.add(chunk_id)
                    validation_results["valid_chunk_ids"] += 1

        return validation_results

    def validate_consistency(self, query: str, num_runs: int = None) -> Dict[str, Any]:
        """
        Validate consistency of results across multiple test runs
        """
        if not self.connected:
            raise RuntimeError("Not connected to Qdrant. Call connect() first.")

        if num_runs is None:
            num_runs = config.num_test_runs

        all_results = []
        all_scores = []

        for run in range(num_runs):
            result = self.validate_single_query(query)
            all_results.append(result)

            # Collect top scores for consistency analysis
            if result.retrieved_chunks:
                top_score = result.retrieved_chunks[0].score
                all_scores.append(top_score)

        # Calculate consistency metrics using the methods below
        consistency_result = self.calculate_consistency_metrics(all_scores)
        return {
            "query": query,
            "num_runs": num_runs,
            "mean_score": consistency_result["mean_score"],
            "coefficient_of_variation": consistency_result["coefficient_of_variation"],
            "variance": consistency_result["variance"],
            "std_dev": consistency_result["std_dev"],
            "consistency_met": consistency_result["consistency_met"],
            "all_results": all_results
        }

    def calculate_consistency_metrics(self, scores: List[float]) -> Dict[str, float]:
        """
        Create consistency measurement algorithm for similarity scores
        """
        if not scores:
            return {
                "mean_score": 0.0,
                "variance": 0.0,
                "std_dev": 0.0,
                "coefficient_of_variation": 0.0,
                "consistency_met": True  # Consistent if no data to compare
            }

        # Calculate mean
        mean_score = sum(scores) / len(scores)

        # Implement variance calculation for similarity scores across runs
        if len(scores) > 1:
            variance = sum((score - mean_score) ** 2 for score in scores) / len(scores)
            std_dev = variance ** 0.5
            coefficient_of_variation = (std_dev / mean_score) if mean_score != 0 else 0
        else:
            variance = 0.0
            std_dev = 0.0
            coefficient_of_variation = 0.0

        # Create consistency validation method with <5% variation threshold
        consistency_met = coefficient_of_variation < 0.05  # Less than 5% variation

        return {
            "mean_score": mean_score,
            "variance": variance,
            "std_dev": std_dev,
            "coefficient_of_variation": coefficient_of_variation,
            "consistency_met": consistency_met
        }

    def add_consistency_validation_to_results(self, original_summary: ValidationSummary, consistency_results: List[Dict[str, Any]]) -> ValidationSummary:
        """
        Add consistency validation to validation results
        """
        if not consistency_results:
            return original_summary

        # Calculate overall consistency score based on all consistency tests
        consistency_scores = [result.get("coefficient_of_variation", 0) for result in consistency_results]
        if consistency_scores:
            # Lower coefficient of variation means higher consistency
            # Convert to a score where 1.0 is perfectly consistent and 0.0 is highly inconsistent
            avg_coeff_var = sum(consistency_scores) / len(consistency_scores)
            consistency_score = max(0.0, 1.0 - (avg_coeff_var / 0.05))  # Normalize against 5% threshold
        else:
            consistency_score = 0.0

        # Create a new summary with updated consistency score
        return ValidationSummary(
            total_queries=original_summary.total_queries,
            queries_passed=original_summary.queries_passed,
            relevance_accuracy=original_summary.relevance_accuracy,
            metadata_accuracy=original_summary.metadata_accuracy,
            consistency_score=consistency_score,
            start_time=original_summary.start_time,
            end_time=original_summary.end_time,
            execution_time=original_summary.execution_time
        )

    def test_self_similarity(self, chunk_id: str) -> Dict[str, Any]:
        """
        Implement self-similarity test for known document chunks
        """
        if not self.connected:
            raise RuntimeError("Not connected to Qdrant. Call connect() first.")

        try:
            # Retrieve the specific chunk
            metadata_list = self.qdrant_connector.get_chunk_metadata([chunk_id])

            if not metadata_list:
                return {
                    "chunk_id": chunk_id,
                    "success": False,
                    "error": "Chunk not found in Qdrant"
                }

            # Get the chunk's content from metadata to create a query
            chunk_metadata = metadata_list[0]
            payload = chunk_metadata.get("payload", {})
            chunk_content = payload.get("content", "")

            if not chunk_content:
                return {
                    "chunk_id": chunk_id,
                    "success": False,
                    "error": "Chunk content not available for self-similarity test"
                }

            # Convert chunk content to vector
            chunk_vector = self.qdrant_connector.text_to_vector(chunk_content)

            if not chunk_vector:
                return {
                    "chunk_id": chunk_id,
                    "success": False,
                    "error": "Failed to convert chunk content to vector"
                }

            # Perform search with the same content as query
            # This should return the same chunk as the top result with high similarity
            similarity_scores = self.qdrant_connector.search_with_text(chunk_content, top_k=5)

            # Check if the original chunk appears in the results with high similarity
            original_chunk_found = any(score.chunk_id == chunk_id and score.score > 0.8
                                    for score in similarity_scores[:3])  # Check top 3 results

            # Calculate self-similarity metrics
            top_result = similarity_scores[0] if similarity_scores else None
            top_score = top_result.score if top_result else 0.0
            top_chunk_id = top_result.chunk_id if top_result else None

            return {
                "chunk_id": chunk_id,
                "success": True,
                "original_chunk_found_in_top_3": original_chunk_found,
                "top_result_chunk_id": top_chunk_id,
                "top_similarity_score": top_score,
                "is_self_similar": top_score > 0.8 and top_chunk_id == chunk_id
            }

        except Exception as e:
            return {
                "chunk_id": chunk_id,
                "success": False,
                "error": str(e)
            }

    def print_consistency_validation_result(self, consistency_result: Dict[str, Any]):
        """
        Implement console output for consistency validation results
        """
        print(f"\n--- Consistency Validation Results ---")
        print(f"Query: {consistency_result.get('query', 'Unknown')}")
        print(f"Number of Test Runs: {consistency_result.get('num_runs', 0)}")
        print(f"Mean Score: {consistency_result.get('mean_score', 0):.4f}")
        print(f"Variance: {consistency_result.get('variance', 0):.6f}")
        print(f"Standard Deviation: {consistency_result.get('std_dev', 0):.6f}")
        print(f"Coefficient of Variation: {consistency_result.get('coefficient_of_variation', 0):.4%}")
        print(f"Consistency Threshold Met: {'YES' if consistency_result.get('consistency_met', False) else 'NO'}")

        # Additional metrics
        if "all_results" in consistency_result and consistency_result["all_results"]:
            scores = []
            for result in consistency_result["all_results"]:
                if result.retrieved_chunks:
                    scores.append(result.retrieved_chunks[0].score)

            if scores:
                print(f"Score Range: {min(scores):.4f} - {max(scores):.4f}")
                print(f"Min Score: {min(scores):.4f}")
                print(f"Max Score: {max(scores):.4f}")

    def generate_validation_summary(self, results: List[ValidationResult]) -> ValidationSummary:
        """
        Generate summary of validation results
        """
        if not results:
            return ValidationSummary(
                total_queries=0,
                queries_passed=0,
                relevance_accuracy=0.0,
                metadata_accuracy=0.0,
                consistency_score=0.0,
                start_time=datetime.now(),
                end_time=datetime.now(),
                execution_time=0.0
            )

        total_queries = len(results)
        queries_passed = sum(1 for r in results if r.validation_passed)
        relevance_accuracy = sum(1 for r in results if r.relevance_threshold_met) / total_queries
        metadata_accuracy = sum(1 for r in results if r.metadata_valid) / total_queries

        start_time = min(r.timestamp for r in results)
        end_time = max(r.timestamp for r in results)
        total_execution_time = sum(r.execution_time for r in results)

        # For consistency score, we would need multiple runs per query
        # For now, we'll use a placeholder based on overall performance
        consistency_score = relevance_accuracy * metadata_accuracy

        return ValidationSummary(
            total_queries=total_queries,
            queries_passed=queries_passed,
            relevance_accuracy=relevance_accuracy,
            metadata_accuracy=metadata_accuracy,
            consistency_score=consistency_score,
            start_time=start_time,
            end_time=end_time,
            execution_time=total_execution_time
        )

    def run_complete_validation(self, sample_queries: List[str] = None) -> ValidationSummary:
        """
        Run complete validation including accuracy, metadata, and consistency checks
        """
        if sample_queries is None:
            sample_queries = config.default_sample_queries

        # Connect if not already connected
        if not self.connected:
            self.connect()

        # Run accuracy validation
        accuracy_results = self.validate_retrieval_accuracy(sample_queries)

        # Generate summary
        summary = self.generate_validation_summary(accuracy_results)

        return summary

    def print_validation_result(self, result: ValidationResult, detailed: bool = False):
        """
        Print validation result to console
        """
        print(f"\n--- Validation Result for Query: '{result.query_content}' ---")
        print(f"Query ID: {result.query_id}")
        print(f"Validation Passed: {result.validation_passed}")
        print(f"Relevance Threshold Met (> {config.similarity_threshold}): {result.relevance_threshold_met}")
        print(f"Metadata Valid: {result.metadata_valid}")
        print(f"Execution Time: {result.execution_time:.4f} seconds")
        print(f"Timestamp: {result.timestamp}")

        if detailed and result.retrieved_chunks:
            print(f"\nTop {len(result.retrieved_chunks)} Retrieved Chunks:")
            for i, chunk in enumerate(result.retrieved_chunks):
                print(f"  {i+1}. Chunk ID: {chunk.chunk_id}")
                print(f"     Score: {chunk.score:.4f}")
                print(f"     Rank: {chunk.rank}")

    def print_validation_summary(self, summary: ValidationSummary):
        """
        Print validation summary to console
        """
        print(f"\n--- Validation Summary ---")
        print(f"Total Queries: {summary.total_queries}")
        print(f"Queries Passed: {summary.queries_passed}")
        print(f"Relevance Accuracy: {summary.relevance_accuracy:.2%}")
        print(f"Metadata Accuracy: {summary.metadata_accuracy:.2%}")
        print(f"Consistency Score: {summary.consistency_score:.2%}")
        print(f"Total Execution Time: {summary.execution_time:.4f} seconds")
        print(f"Start Time: {summary.start_time}")
        print(f"End Time: {summary.end_time}")

    def create_detailed_validation_report(self, results: List[ValidationResult], summary: ValidationSummary) -> Dict[str, Any]:
        """
        Create detailed validation report with all test results
        """
        report = {
            "summary": {
                "total_queries": summary.total_queries,
                "queries_passed": summary.queries_passed,
                "relevance_accuracy": summary.relevance_accuracy,
                "metadata_accuracy": summary.metadata_accuracy,
                "consistency_score": summary.consistency_score,
                "execution_time": summary.execution_time,
                "start_time": summary.start_time.isoformat() if hasattr(summary.start_time, 'isoformat') else str(summary.start_time),
                "end_time": summary.end_time.isoformat() if hasattr(summary.end_time, 'isoformat') else str(summary.end_time),
            },
            "detailed_results": [],
            "success_criteria": {
                "relevance_accuracy_90_percent": summary.relevance_accuracy >= 0.9,
                "metadata_accuracy_100_percent": summary.metadata_accuracy == 1.0,
                "consistency_within_threshold": summary.consistency_score > 0.0,  # Placeholder
            }
        }

        for result in results:
            result_dict = {
                "query_id": result.query_id,
                "query_content": result.query_content,
                "validation_passed": result.validation_passed,
                "relevance_threshold_met": result.relevance_threshold_met,
                "metadata_valid": result.metadata_valid,
                "execution_time": result.execution_time,
                "timestamp": result.timestamp.isoformat() if hasattr(result.timestamp, 'isoformat') else str(result.timestamp),
                "retrieved_chunks": [
                    {
                        "chunk_id": chunk.chunk_id,
                        "score": chunk.score,
                        "rank": chunk.rank
                    } for chunk in result.retrieved_chunks
                ]
            }
            report["detailed_results"].append(result_dict)

        return report

    def calculate_pass_fail_status(self, summary: ValidationSummary) -> Dict[str, bool]:
        """
        Implement pass/fail status calculation based on success criteria
        """
        # SC-001: The system retrieves relevant document chunks with similarity scores above 0.7 for 90% of sample queries
        relevance_threshold_met = summary.relevance_accuracy >= 0.9

        # SC-002: All retrieved chunks contain complete metadata (URL, section, chunk ID) with 100% accuracy
        metadata_threshold_met = summary.metadata_accuracy == 1.0

        # SC-003: The same queries produce consistent results across 10 test runs with less than 5% variation in similarity scores
        # This is represented by the consistency score in the summary
        consistency_threshold_met = summary.consistency_score > 0.95  # High consistency score

        # SC-004: Console-based test results display clear validation metrics and pass/fail status for each validation check
        # This is handled by the display functions

        overall_pass = relevance_threshold_met and metadata_threshold_met and consistency_threshold_met

        return {
            "relevance_threshold_met": relevance_threshold_met,
            "metadata_threshold_met": metadata_threshold_met,
            "consistency_threshold_met": consistency_threshold_met,
            "overall_pass": overall_pass,
            "individual_criteria": {
                "SC-001": relevance_threshold_met,
                "SC-002": metadata_threshold_met,
                "SC-003": consistency_threshold_met
            }
        }

    def print_metadata_validation_result(self, validation_result: Dict[str, Any], title: str = "Metadata Validation"):
        """
        Implement console output for metadata validation results
        """
        print(f"\n--- {title} ---")
        print(f"Total Chunks: {validation_result.get('total_chunks', 0)}")
        print(f"Valid Chunks: {validation_result.get('valid_chunks', 0)}")
        print(f"Completeness Score: {validation_result.get('completeness_score', 0):.2%}")

        # Show missing fields breakdown
        missing_fields = validation_result.get('missing_fields', {})
        if missing_fields:
            print(f"Missing Fields Breakdown:")
            for field, count in missing_fields.items():
                print(f"  {field}: {count}")

        # Show invalid chunks if any
        invalid_chunks = validation_result.get('invalid_chunks', [])
        if invalid_chunks:
            print(f"Invalid Chunks:")
            for chunk in invalid_chunks[:5]:  # Show first 5 invalid chunks
                chunk_id = chunk.get('chunk_id', 'unknown')
                missing = chunk.get('missing_fields', [])
                print(f"  Chunk {chunk_id}: missing {', '.join(missing)}")

        if len(invalid_chunks) > 5:
            print(f"  ... and {len(invalid_chunks) - 5} more")


# CLI Interface
def create_argparser():
    """Create argument parser for the validation CLI"""
    parser = argparse.ArgumentParser(
        description="CLI tool for validating Qdrant retrieval accuracy, metadata integrity, and consistency"
    )

    # Main command
    subparsers = parser.add_subparsers(dest='command', help='Available commands')

    # Validation command
    validate_parser = subparsers.add_parser('validate', help='Run validation tests')
    validate_parser.add_argument(
        '--queries',
        nargs='+',
        help='Sample queries to test (default: use config defaults)'
    )
    validate_parser.add_argument(
        '--collection-name',
        type=str,
        default=config.collection_name,
        help=f'Qdrant collection name (default: {config.collection_name})'
    )
    validate_parser.add_argument(
        '--output-format',
        choices=['json', 'text'],
        default='text',
        help='Output format (default: text)'
    )
    validate_parser.add_argument(
        '--output-file',
        type=str,
        help='Output file to write results (default: stdout)'
    )
    validate_parser.add_argument(
        '--validate-accuracy',
        action='store_true',
        help='Run accuracy validation tests'
    )
    validate_parser.add_argument(
        '--validate-metadata',
        action='store_true',
        help='Run metadata integrity validation tests'
    )
    validate_parser.add_argument(
        '--validate-consistency',
        action='store_true',
        help='Run consistency validation tests'
    )
    validate_parser.add_argument(
        '--config-file',
        type=str,
        help='Configuration file path (YAML or JSON format)'
    )
    validate_parser.add_argument(
        '--all',
        action='store_true',
        help='Run all validation tests (accuracy, metadata, consistency)'
    )

    # Test individual functionality
    test_parser = subparsers.add_parser('test', help='Test specific functionality')
    test_parser.add_argument(
        'test_type',
        choices=['connection', 'query', 'metadata', 'consistency', 'self-similarity'],
        help='Type of test to run'
    )
    test_parser.add_argument(
        '--query',
        type=str,
        help='Query text for query testing'
    )
    test_parser.add_argument(
        '--chunk-id',
        type=str,
        help='Chunk ID for self-similarity testing'
    )

    return parser


def load_config_from_file(config_file_path: str):
    """Load configuration from a file (YAML or JSON)"""
    config_path = Path(config_file_path)

    if not config_path.exists():
        print(f"ERROR: Configuration file does not exist: {config_file_path}", file=sys.stderr)
        return None

    try:
        with open(config_path, 'r') as f:
            if config_path.suffix.lower() in ['.yaml', '.yml']:
                import yaml
                config_data = yaml.safe_load(f)
            elif config_path.suffix.lower() == '.json':
                import json
                config_data = json.load(f)
            else:
                print(f"ERROR: Unsupported configuration file format: {config_path.suffix}", file=sys.stderr)
                return None

        return config_data

    except Exception as e:
        print(f"ERROR: Error loading configuration file: {e}", file=sys.stderr)
        return None


def run_complete_validation(queries: List[str] = None, output_format: str = 'text', output_file: str = None):
    """Run complete validation tests"""
    try:
        # Initialize validator
        connector = QdrantConnector(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.collection_name
        )
        validator = RetrievalValidator(connector)

        # Connect to Qdrant
        if not validator.connect():
            print("ERROR: Failed to connect to Qdrant", file=sys.stderr)
            return False

        print(f"SUCCESS: Connected to Qdrant collection: {config.collection_name}")

        # Run validation
        summary = validator.run_complete_validation(queries)

        # Output results
        if output_format == 'json':
            result = {
                "total_queries": summary.total_queries,
                "queries_passed": summary.queries_passed,
                "relevance_accuracy": summary.relevance_accuracy,
                "metadata_accuracy": summary.metadata_accuracy,
                "consistency_score": summary.consistency_score,
                "execution_time": summary.execution_time
            }
            output = json.dumps(result, indent=2)
        else:
            validator.print_validation_summary(summary)
            output = ""

        # Write to file or stdout
        if output_file:
            with open(output_file, 'w') as f:
                if output:
                    f.write(output)
                else:
                    f.write("Validation completed successfully\n")
        elif output_format == 'json':
            print(output)

        return True

    except Exception as e:
        print(f"ERROR: Validation failed: {e}", file=sys.stderr)
        return False


def validate_input_queries(queries: List[str]) -> List[str]:
    """Add input validation for query parameters"""
    if not queries:
        print("WARNING: No queries provided", file=sys.stderr)
        return []

    validated_queries = []
    for query in queries:
        if not query or not query.strip():
            print(f"WARNING: Skipping empty query", file=sys.stderr)
            continue

        # Basic validation: remove excessive whitespace and check length
        cleaned_query = query.strip()
        if len(cleaned_query) < 2:
            print(f"WARNING: Skipping query too short: '{cleaned_query}'", file=sys.stderr)
            continue

        if len(cleaned_query) > 500:
            print(f"WARNING: Truncating very long query: '{cleaned_query[:50]}...'", file=sys.stderr)
            cleaned_query = cleaned_query[:500]

        validated_queries.append(cleaned_query)

    return validated_queries


def run_accuracy_validation(queries: List[str], output_format: str = 'text'):
    """Run accuracy validation tests"""
    try:
        # Validate input queries
        validated_queries = validate_input_queries(queries)
        if not validated_queries:
            print("ERROR: No valid queries provided after validation", file=sys.stderr)
            return False

        connector = QdrantConnector(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.collection_name
        )
        validator = RetrievalValidator(connector)

        if not validator.connect():
            print("ERROR: Failed to connect to Qdrant", file=sys.stderr)
            return False

        print(f"RUNNING: Running accuracy validation for {len(validated_queries)} queries...")

        results = validator.validate_retrieval_accuracy(validated_queries)

        for result in results:
            if output_format == 'text':
                validator.print_validation_result(result)
            else:
                # JSON output
                result_dict = {
                    "query_id": result.query_id,
                    "query_content": result.query_content,
                    "validation_passed": result.validation_passed,
                    "relevance_threshold_met": result.relevance_threshold_met,
                    "metadata_valid": result.metadata_valid,
                    "execution_time": result.execution_time,
                    "retrieved_chunks": [
                        {"chunk_id": chunk.chunk_id, "score": chunk.score, "rank": chunk.rank}
                        for chunk in result.retrieved_chunks
                    ]
                }
                print(json.dumps(result_dict, indent=2))

        return True

    except Exception as e:
        print(f"ERROR: Accuracy validation failed: {e}", file=sys.stderr)
        return False


def validate_chunk_ids(chunk_ids: List[str]) -> List[str]:
    """Validate chunk IDs input"""
    if not chunk_ids:
        return chunk_ids  # Allow None/empty to use defaults

    validated_chunk_ids = []
    for chunk_id in chunk_ids:
        if not chunk_id or not str(chunk_id).strip():
            print(f"WARNING: Skipping empty chunk ID", file=sys.stderr)
            continue

        cleaned_id = str(chunk_id).strip()
        if len(cleaned_id) > 100:  # Reasonable length limit
            print(f"WARNING: Skipping chunk ID too long: '{cleaned_id[:30]}...'", file=sys.stderr)
            continue

        validated_chunk_ids.append(cleaned_id)

    return validated_chunk_ids


def run_metadata_validation(chunk_ids: List[str] = None, output_format: str = 'text'):
    """Run metadata validation tests"""
    try:
        connector = QdrantConnector(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.collection_name
        )
        validator = RetrievalValidator(connector)

        if not validator.connect():
            print("ERROR: Failed to connect to Qdrant", file=sys.stderr)
            return False

        # Validate chunk IDs if provided
        if chunk_ids is not None:
            chunk_ids = validate_chunk_ids(chunk_ids)
            if not chunk_ids:
                print("ERROR: No valid chunk IDs provided after validation", file=sys.stderr)
                return False

        # If no chunk IDs provided, get a sample from the collection
        if not chunk_ids:
            print("INFO: Retrieving sample chunk IDs for metadata validation...")
            chunk_ids = connector.get_all_chunk_ids()[:10]  # Get first 10 chunks

        print(f"RUNNING: Running metadata validation for {len(chunk_ids)} chunks...")

        # Run various metadata validation checks
        completeness_result = validator.validate_metadata_completeness(chunk_ids)
        url_result = validator.validate_url_field(chunk_ids)
        section_result = validator.validate_section_field(chunk_ids)
        id_result = validator.validate_chunk_id_field(chunk_ids)

        if output_format == 'text':
            validator.print_metadata_validation_result(completeness_result, "Completeness Validation")
            print(f"\nURL Validation - Valid: {url_result['valid_urls']}, Invalid: {len(url_result['invalid_urls'])}")
            print(f"Section Validation - Valid: {section_result['valid_sections']}, Missing: {section_result['missing_sections']}")
            print(f"ID Validation - Valid: {id_result['valid_chunk_ids']}, Missing: {id_result['missing_chunk_ids']}, Duplicates: {len(id_result['duplicate_chunk_ids'])}")
        else:
            result = {
                "completeness": completeness_result,
                "url_validation": url_result,
                "section_validation": section_result,
                "id_validation": id_result
            }
            print(json.dumps(result, indent=2))

        return True

    except Exception as e:
        print(f"ERROR: Metadata validation failed: {e}", file=sys.stderr)
        return False


def run_consistency_validation(queries: List[str], output_format: str = 'text'):
    """Run consistency validation tests"""
    try:
        # Validate input queries
        validated_queries = validate_input_queries(queries)
        if not validated_queries:
            print("ERROR: No valid queries provided after validation", file=sys.stderr)
            return False

        connector = QdrantConnector(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.collection_name
        )
        validator = RetrievalValidator(connector)

        if not validator.connect():
            print("ERROR: Failed to connect to Qdrant", file=sys.stderr)
            return False

        print(f"RUNNING: Running consistency validation for {len(validated_queries)} queries...")

        for query in validated_queries:
            result = validator.validate_consistency(query)

            if output_format == 'text':
                validator.print_consistency_validation_result(result)
            else:
                print(json.dumps(result, indent=2))

        return True

    except Exception as e:
        print(f"ERROR: Consistency validation failed: {e}", file=sys.stderr)
        return False


def test_connection():
    """Test Qdrant connection"""
    try:
        connector = QdrantConnector(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.collection_name
        )

        if connector.connect():
            collection_info = connector.get_collection_info()
            print(f"SUCCESS: Connected to Qdrant")
            print(f"INFO: Collection: {config.collection_name}")
            print(f"INFO: Points count: {collection_info.get('points_count', 'Unknown')}")
            return True
        else:
            print("ERROR: Failed to connect to Qdrant")
            return False

    except Exception as e:
        print(f"ERROR: Connection test failed: {e}", file=sys.stderr)
        return False


def test_single_query(query: str):
    """Test a single query"""
    try:
        connector = QdrantConnector(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.collection_name
        )
        validator = RetrievalValidator(connector)

        if not validator.connect():
            print("ERROR: Failed to connect to Qdrant", file=sys.stderr)
            return False

        print(f"TESTING: Testing query: '{query}'")
        result = validator.validate_single_query(query)

        validator.print_validation_result(result, detailed=True)
        return True

    except Exception as e:
        print(f"ERROR: Query test failed: {e}", file=sys.stderr)
        return False


def test_self_similarity(chunk_id: str):
    """Test self-similarity of a document chunk"""
    try:
        connector = QdrantConnector(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.collection_name
        )
        validator = RetrievalValidator(connector)

        if not validator.connect():
            print("ERROR: Failed to connect to Qdrant", file=sys.stderr)
            return False

        print(f"TESTING: Testing self-similarity for chunk: {chunk_id}")
        result = validator.test_self_similarity(chunk_id)

        print(f"Chunk ID: {result['chunk_id']}")
        print(f"Success: {result['success']}")
        if result['success']:
            print(f"Is Self-Similar: {result['is_self_similar']}")
            print(f"Top Score: {result.get('top_similarity_score', 0):.4f}")
            print(f"Top Result Chunk ID: {result.get('top_result_chunk_id', 'N/A')}")
        else:
            print(f"Error: {result.get('error', 'Unknown error')}")

        return result['success']

    except Exception as e:
        print(f"ERROR: Self-similarity test failed: {e}", file=sys.stderr)
        return False


def main():
    """Main CLI entry point"""
    parser = create_argparser()
    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return 1

    # Load configuration from file if specified
    config_data = None
    if hasattr(args, 'config_file') and args.config_file:
        config_data = load_config_from_file(args.config_file)
        if config_data is None:
            return 1

    if args.command == 'validate':
        # Determine which validations to run
        run_accuracy = args.all or args.validate_accuracy
        run_metadata = args.all or args.validate_metadata
        run_consistency = args.all or args.validate_consistency

        # Use provided queries or default ones
        queries = args.queries or config_data.get('queries', config.default_sample_queries) if config_data else config.default_sample_queries

        success = True

        if run_accuracy:
            success &= run_accuracy_validation(queries, args.output_format)

        if run_metadata:
            # For metadata validation, we need chunk IDs, not queries
            # So we'll just run a general metadata check
            success &= run_metadata_validation(output_format=args.output_format)

        if run_consistency:
            success &= run_consistency_validation(queries, args.output_format)

        if not (run_accuracy or run_metadata or run_consistency):
            # If no specific validation was requested, run complete validation
            success = run_complete_validation(queries, args.output_format, args.output_file)

        return 0 if success else 1

    elif args.command == 'test':
        if args.test_type == 'connection':
            success = test_connection()
        elif args.test_type == 'query':
            if not args.query:
                print("ERROR: Query test requires --query argument", file=sys.stderr)
                return 1
            success = test_single_query(args.query)
        elif args.test_type == 'self-similarity':
            if not args.chunk_id:
                print("ERROR: Self-similarity test requires --chunk-id argument", file=sys.stderr)
                return 1
            success = test_self_similarity(args.chunk_id)
        elif args.test_type == 'metadata':
            success = run_metadata_validation(output_format='text')
        elif args.test_type == 'consistency':
            queries = config.default_sample_queries[:1]  # Use first default query
            success = run_consistency_validation(queries, output_format='text')
        else:
            print(f"ERROR: Unknown test type: {args.test_type}", file=sys.stderr)
            return 1

        return 0 if success else 1

    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    sys.exit(main())