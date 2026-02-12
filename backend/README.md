# Website Content Ingestion and Retrieval Validation Pipeline

A comprehensive system that includes both a content ingestion pipeline and a validation tool. The ingestion pipeline crawls Docusaurus documentation sites, extracts clean text, generates Cohere embeddings, and stores them with metadata in Qdrant vector database. The validation tool connects to the Qdrant collection to test retrieval accuracy, metadata integrity, and consistency of vector-based search.

## Features

### Content Ingestion Pipeline
- Crawls Docusaurus documentation sites
- Extracts clean text content from web pages
- Generates semantic embeddings using Cohere
- Stores embeddings with metadata in Qdrant vector database
- Command-line interface for easy execution
- Progress tracking and logging

### Retrieval Validation Tool
- **Qdrant Retrieval Validation**: Tests that the retrieval system returns relevant document chunks for sample queries
- **Metadata Integrity Verification**: Confirms that retrieved chunks contain complete metadata (URL, section, chunk ID)
- **Consistency Testing**: Validates that the embedding process and similarity search produce consistent results across multiple test runs
- **Console-based Output**: Provides clear validation metrics and pass/fail status

## Prerequisites

- Python 3.11 or higher
- uv package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. Clone the repository
2. Create a virtual environment and activate it
3. Install dependencies using uv:
   ```bash
   uv pip install -r requirements.txt
   ```
   Or using pip:
   ```bash
   pip install -e .
   ```

4. Copy the example environment file:
   ```bash
   cp .env.example .env
   ```

5. Update `.env` with your actual API keys and URLs:
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_URL`: Your Qdrant Cloud instance URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `SOURCE_URL`: The Docusaurus site URL to ingest

## Usage

### Content Ingestion Pipeline

Run the ingestion pipeline with the following command:

```bash
py --python main.source-url https://your-docusaurus-site.com --chunk-size 512 --overlap-size 102
```

#### Command Line Arguments

- `--source-url` (required): URL of the Docusaurus site to ingest
- `--chunk-size` (optional): Size of text chunks in tokens (default: 512)
- `--overlap-size` (optional): Overlap size between chunks in tokens (default: 102)

### Retrieval Validation Tool

#### Run Complete Validation

```bash
python -m src.cli.validation_cli validate --all
```

#### Run Specific Validation Types

Run accuracy validation only:
```bash
python -m src.cli.validation_cli validate --validate-accuracy
```

Run metadata validation only:
```bash
python -m src.cli.validation_cli validate --validate-metadata
```

Run consistency validation only:
```bash
python -m src.cli.validation_cli validate --validate-consistency
```

#### Use Custom Queries

```bash
python -m src.cli.validation_cli validate --queries "vector embeddings" "Qdrant configuration" --all
```

#### Specify Output Format

```bash
python -m src.cli.validation_cli validate --all --output-format json
```

#### Use Configuration File

```bash
python -m src.cli.validation_cli validate --config-file config.yaml --all
```

#### Test Commands

Test Qdrant connection:
```bash
python -m src.cli.validation_cli test connection
```

Test single query:
```bash
python -m src.cli.validation_cli test query --query "your query text"
```

Test self-similarity:
```bash
python -m src.cli.validation_cli test self-similarity --chunk-id "chunk_id_to_test"
```

## Configuration File Format (YAML or JSON)

Example `config.yaml`:

```yaml
queries:
  - "vector embeddings"
  - "Qdrant configuration"
  - "retrieval accuracy"
output_format: "text"
validate_accuracy: true
validate_metadata: true
validate_consistency: true
```

## Architecture

### Content Ingestion Pipeline
The pipeline consists of three main phases:

1. **Content Crawling and Extraction**: Crawls the specified Docusaurus site and extracts clean text content
2. **Embedding Generation**: Chunks the text and generates semantic embeddings using Cohere
3. **Vector Storage**: Stores the embeddings with metadata in Qdrant vector database

### Retrieval Validation Tool
The validation system is organized into the following modules:

- `src/retrieval/`: Core retrieval validation logic
  - `validator.py`: Main validation logic and orchestrator
  - `qdrant_connector.py`: Qdrant database interaction
  - `models.py`: Data models for validation results
  - `config.py`: Configuration management
- `src/cli/`: Command-line interface
  - `validation_cli.py`: CLI interface for validation

## Success Criteria

The validation tool checks against these success criteria:

1. **SC-001**: The system retrieves relevant document chunks with similarity scores above 0.7 for 90% of sample queries
2. **SC-002**: All retrieved chunks contain complete metadata (URL, section, chunk ID) with 100% accuracy
3. **SC-003**: The same queries produce consistent results across 10 test runs with less than 5% variation in similarity scores
4. **SC-004**: Console-based test results display clear validation metrics and pass/fail status for each validation check

## Security

- API keys are loaded from environment variables
- Input URLs are validated before processing
- Content is sanitized during extraction

## License

[Specify your license here]