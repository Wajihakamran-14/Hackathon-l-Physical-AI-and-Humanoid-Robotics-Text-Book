"""
Sample query execution for testing retrieval validation
"""
from retrieval import RetrievalValidator, QdrantConnector
import json


def test_vector_embeddings_query():
    """
    Test case for 'vector embeddings' query
    """
    print("Testing 'vector embeddings' query...")

    # Initialize validator
    connector = QdrantConnector()
    validator = RetrievalValidator(connector)

    # Connect to Qdrant
    if not validator.connect():
        print("Failed to connect to Qdrant")
        return None

    # Run validation for "vector embeddings" query
    result = validator.validate_single_query("vector embeddings")

    print(f"Query: {result.query_content}")
    print(f"Validation passed: {result.validation_passed}")
    print(f"Relevance threshold met: {result.relevance_threshold_met}")
    print(f"Metadata valid: {result.metadata_valid}")
    print(f"Number of retrieved chunks: {len(result.retrieved_chunks)}")

    if result.retrieved_chunks:
        print(f"Top result - Chunk ID: {result.retrieved_chunks[0].chunk_id}, Score: {result.retrieved_chunks[0].score}")

    return result


def test_qdrant_configuration_query():
    """
    Test case for 'Qdrant configuration' query
    """
    print("\nTesting 'Qdrant configuration' query...")

    # Initialize validator
    connector = QdrantConnector()
    validator = RetrievalValidator(connector)

    # Connect to Qdrant
    if not validator.connect():
        print("Failed to connect to Qdrant")
        return None

    # Run validation for "Qdrant configuration" query
    result = validator.validate_single_query("Qdrant configuration")

    print(f"Query: {result.query_content}")
    print(f"Validation passed: {result.validation_passed}")
    print(f"Relevance threshold met: {result.relevance_threshold_met}")
    print(f"Metadata valid: {result.metadata_valid}")
    print(f"Number of retrieved chunks: {len(result.retrieved_chunks)}")

    if result.retrieved_chunks:
        print(f"Top result - Chunk ID: {result.retrieved_chunks[0].chunk_id}, Score: {result.retrieved_chunks[0].score}")

    return result


def run_sample_queries():
    """
    Run both sample query test cases
    """
    print("Running sample query validation tests...\n")

    result1 = test_vector_embeddings_query()
    result2 = test_qdrant_configuration_query()

    print("\nSample query execution completed.")

    return result1, result2


if __name__ == "__main__":
    run_sample_queries()