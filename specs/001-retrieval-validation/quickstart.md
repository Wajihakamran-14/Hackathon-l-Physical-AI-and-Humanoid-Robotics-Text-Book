# Quickstart: Retrieval Pipeline Validation

## Prerequisites
- Python 3.11+
- Access to Qdrant instance with existing collection from Spec 1
- Environment with required dependencies installed

## Setup

1. Install dependencies:
```bash
pip install qdrant-client python-dotenv numpy
```

2. Set up environment variables:
```bash
# Create .env file with Qdrant connection details
QDRANT_HOST=your-qdrant-host
QDRANT_PORT=6333
QDRANT_API_KEY=your-api-key-if-required
COLLECTION_NAME=your-collection-name
```

## Basic Usage

### Run Validation Tests
```bash
# Run validation with default settings
python -m src.cli.validation_cli --queries "vector embeddings, Qdrant configuration" --threshold 0.7

# Run validation with custom configuration
python -m src.cli.validation_cli --config validation_config.json --output results.json
```

### Validation Configuration
Create a `validation_config.json` file:
```json
{
  "queries": [
    {
      "id": "query-1",
      "content": "vector embeddings"
    },
    {
      "id": "query-2",
      "content": "Qdrant configuration"
    }
  ],
  "validation_config": {
    "similarity_threshold": 0.7,
    "num_test_runs": 10,
    "validate_metadata": true,
    "collection_name": "existing-collection"
  }
}
```

## Output Format
The validation tool outputs results in JSON format:
```json
{
  "validation_id": "uuid",
  "status": "completed",
  "summary": {
    "total_queries": 2,
    "queries_passed": 2,
    "relevance_accuracy": 1.0,
    "metadata_accuracy": 1.0,
    "consistency_score": 0.95
  },
  "results": [
    // Detailed results for each query
  ]
}
```

## Example Validation
```python
from src.retrieval.validator import RetrievalValidator

# Initialize validator
validator = RetrievalValidator(
    qdrant_host="localhost",
    qdrant_port=6333,
    collection_name="existing-collection"
)

# Run validation
results = validator.validate_queries([
    {"id": "test-1", "content": "vector embeddings"},
    {"id": "test-2", "content": "Qdrant configuration"}
])

print(f"Validation accuracy: {results.summary.relevance_accuracy}")
```