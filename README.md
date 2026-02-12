# RAG Agent API with FastAPI Integration

A Retrieval-Augmented Generation (RAG) system that connects to Qdrant for knowledge retrieval and integrates with OpenRouter for LLM-based responses. The system includes a FastAPI backend to expose the RAG agent through API endpoints for frontend integration.

## Features

- FastAPI server running locally
- Single API endpoint for processing user queries
- Integration with existing RAG agent logic
- Retrieval-augmented responses using Qdrant knowledge base
- OpenRouter-based LLM for response generation
- Validation system for retrieval accuracy, metadata integrity, and consistency

## Prerequisites

- Python 3.11+
- Access to Qdrant instance with existing collection
- Cohere API key for text embeddings
- Environment with required dependencies installed

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables in `.env`:
```bash
# Create .env file with Qdrant connection details
QDRANT_URL=your-qdrant-url
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION=your-collection-name
COHERE_API_KEY=your-cohere-api-key
```

## Basic Usage

### Run Complete Validation
```bash
python backend/retrieval.py validate --all
```

### Run Specific Validation Tests
```bash
# Test retrieval accuracy
python backend/retrieval.py validate --validate-accuracy --queries "vector embeddings" "Qdrant configuration"

# Test metadata integrity
python backend/retrieval.py validate --validate-metadata

# Test consistency
python backend/retrieval.py validate --validate-consistency
```

### Test Individual Functionality
```bash
# Test Qdrant connection
python backend/retrieval.py test connection

# Test single query
python backend/retrieval.py test query --query "vector embeddings"

# Test self-similarity
python backend/retrieval.py test self-similarity --chunk-id "your-chunk-id"
```

### Configuration
Use configuration files to customize validation parameters:

YAML config (`backend/config_examples/validation_config.yaml`):
```yaml
queries:
  - "vector embeddings"
  - "Qdrant configuration"
validation_settings:
  similarity_threshold: 0.7
  num_test_runs: 10
```

JSON config (`backend/config_examples/validation_config.json`):
```json
{
  "queries": ["vector embeddings", "Qdrant configuration"],
  "validation_settings": {
    "similarity_threshold": 0.7,
    "num_test_runs": 10
  }
}
```

Load config file:
```bash
python backend/retrieval.py validate --config-file backend/config_examples/validation_config.yaml
```

## Output Formats
- Text (default): Human-readable console output
- JSON: Machine-readable format for integration

```bash
python backend/retrieval.py validate --output-format json
```

## Features

- **Retrieval Accuracy Validation**: Tests that the system returns relevant document chunks with similarity scores above 0.7 for 90% of sample queries
- **Metadata Integrity Validation**: Confirms that all retrieved chunks contain complete metadata (URL, section, chunk ID) with 100% accuracy
- **Consistency Validation**: Validates that the same queries produce consistent results across 10 test runs with less than 5% variation in similarity scores
- **Comprehensive Reporting**: Provides clear validation metrics and pass/fail status for each validation check

## Configuration Examples

The `backend/config_examples/` directory contains example configuration files for different validation scenarios:

- `validation_config.json` - Basic validation configuration
- `validation_config.yaml` - YAML format configuration
- `advanced_validation_config.yaml` - Advanced configuration with detailed settings

## FastAPI Integration

The system includes a FastAPI server to expose the RAG agent through API endpoints:

- **api.py**: FastAPI server implementation
- **start_api.py**: Script to start the API server
- **Endpoints**:
  - `POST /query` - Process user queries through the RAG agent
  - `GET /health` - Health check endpoint
  - `GET /` - API information and documentation

### Setup and Usage

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables in a `.env` file:
   ```
   OPENROUTER_API_KEY=your_openrouter_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION=your_collection_name
   ```

3. Start the API server:
   ```bash
   python backend/start_api.py
   ```

4. Example API call:
   ```bash
   curl -X POST http://127.0.0.1:8000/query \
     -H "Content-Type: application/json" \
     -d '{
       "query": "What is ROS 2?"
     }'
   ```

## Architecture

The system includes:

- **Data models** (DocumentChunk, QueryVector, SimilarityScore, ValidationResult, ValidationSummary) in `backend/retrieval.py`
- **Qdrant connector** with cosine similarity calculation
- **Retrieval validator** with accuracy, metadata, and consistency validation
- **RAG agent** in `backend/agent.py` with OpenRouter integration
- **FastAPI server** in `backend/api.py` for API endpoints
- **Configuration management** and CLI interfaces