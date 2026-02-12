# Validation API Contract

## Overview
This contract defines the interface for the retrieval validation system.

## Endpoints (Conceptual - CLI-based system)

### Validation Execution
- **Command**: `validate`
- **Description**: Execute validation tests on the retrieval system
- **Parameters**:
  - `--queries`: List of sample queries to test
  - `--validate-accuracy`: Run accuracy validation
  - `--validate-metadata`: Run metadata integrity validation
  - `--validate-consistency`: Run consistency validation
  - `--all`: Run all validation tests
  - `--output-format`: Output format (text or json)
  - `--config-file`: Configuration file path

### Test Execution
- **Command**: `test`
- **Description**: Test specific functionality
- **Subcommands**:
  - `connection`: Test Qdrant connection
  - `query`: Test single query execution
  - `metadata`: Test metadata validation
  - `consistency`: Test consistency validation
  - `self-similarity`: Test self-similarity of document chunks

## Input Schema
```json
{
  "queries": ["string"],
  "validation_config": {
    "similarity_threshold": "float",
    "num_test_runs": "int",
    "validate_metadata": "bool"
  },
  "qdrant_config": {
    "url": "string",
    "api_key": "string",
    "collection_name": "string"
  }
}
```

## Output Schema
```json
{
  "total_queries": "int",
  "queries_passed": "int",
  "relevance_accuracy": "float",
  "metadata_accuracy": "float",
  "consistency_score": "float",
  "execution_time": "float",
  "results": [
    {
      "query_id": "string",
      "query_content": "string",
      "validation_passed": "bool",
      "relevance_threshold_met": "bool",
      "metadata_valid": "bool",
      "retrieved_chunks": [
        {
          "chunk_id": "string",
          "score": "float",
          "rank": "int"
        }
      ]
    }
  ]
}
```