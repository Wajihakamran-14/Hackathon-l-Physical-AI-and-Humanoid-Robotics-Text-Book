# Data Model: OpenAI Agent RAG Implementation

## Entity Models

### 1. RetrievalResult (Pydantic BaseModel)
- **Purpose**: Structure for retrieval results returned by the agent tool
- **Fields**:
  - `query: str` - The original search query
  - `results: List[Dict[str, Any]]` - List of retrieved results with content and metadata
  - `total_found: int` - Total number of results found
  - `relevant_count: int` - Number of results meeting relevance threshold

### 2. DocumentChunk (Dataclass)
- **Purpose**: Represents a segment of content from source documents with associated vector embeddings
- **Fields**:
  - `id: str` - Unique identifier for the document chunk
  - `content: str` - The actual text content of the document chunk
  - `url: str` - Source URL where the original document is located
  - `section: str` - Section identifier within the source document
  - `chunk_id: str` - Unique identifier for this specific chunk
  - `vector: List[float]` - Vector embedding representation of the content
  - `metadata: Dict[str, Any]` - Additional metadata associated with the chunk

### 3. QueryVector (Dataclass)
- **Purpose**: Represents a vectorized query for similarity search in Qdrant
- **Fields**:
  - `query_text: str` - Original text of the query
  - `vector: List[float]` - Vector embedding of the query text
  - `query_id: str` - Unique identifier for this query

### 4. SimilarityScore (Dataclass)
- **Purpose**: Represents the similarity score between a query and a document chunk
- **Fields**:
  - `chunk_id: str` - ID of the document chunk being compared
  - `score: float` - Similarity score (0.0 to 1.0, where 1.0 is most similar)
  - `rank: int` - Ranking position of this result (1 = highest similarity)

### 5. ValidationResult (Dataclass)
- **Purpose**: Represents the result of a single validation test
- **Fields**:
  - `query_id: str` - Unique identifier for the query that was validated
  - `query_content: str` - The original query text that was validated
  - `retrieved_chunks: List[SimilarityScore]` - List of chunks retrieved with their similarity scores
  - `metadata_valid: bool` - Whether the metadata of retrieved chunks is valid
  - `relevance_threshold_met: bool` - Whether relevance threshold was met
  - `validation_passed: bool` - Whether the entire validation test passed
  - `timestamp: datetime` - When the validation was performed
  - `execution_time: float` - Time taken to perform the validation in seconds

### 6. ValidationSummary (Dataclass)
- **Purpose**: Represents a summary of multiple validation tests
- **Fields**:
  - `total_queries: int` - Total number of queries validated
  - `queries_passed: int` - Number of queries that passed validation
  - `relevance_accuracy: float` - Percentage of queries that met relevance threshold
  - `metadata_accuracy: float` - Percentage of queries with valid metadata
  - `consistency_score: float` - Score representing consistency of results
  - `start_time: datetime` - When validation started
  - `end_time: datetime` - When validation ended
  - `execution_time: float` - Total time taken for all validations

## Class Models

### 1. RetrievalAugmentedAgent
- **Purpose**: Main class that implements the retrieval-augmented agent using OpenAI Agent SDK
- **Attributes**:
  - `config: ValidationConfig` - Configuration object for validation settings
  - `similarity_threshold: float` - Minimum similarity score for relevant results
  - `agent: Agent` - The OpenAI Agent instance (or mock if SDK not available)
- **Methods**:
  - `__init__(collection_name: str = None, similarity_threshold: float = 0.7)` - Constructor
  - `query(user_question: str) -> str` - Process user question synchronously
  - `query_async(user_question: str) -> str` - Process user question asynchronously
  - `_basic_retrieve_and_answer(user_question: str) -> str` - Fallback retrieval method

### 2. ValidationConfig
- **Purpose**: Configuration class for retrieval validation
- **Attributes**:
  - `qdrant_url: str` - Qdrant instance URL
  - `qdrant_api_key: str` - Qdrant API key (if required)
  - `collection_name: str` - Name of the Qdrant collection to use
  - `similarity_threshold: float` - Minimum similarity threshold
  - `num_test_runs: int` - Number of test runs for consistency validation
  - `validate_metadata: bool` - Whether to validate metadata
  - `max_memory_usage: int` - Maximum memory usage in MB
  - `max_query_time: float` - Maximum query time in seconds
  - `default_sample_queries: List[str]` - Default queries for validation

### 3. QdrantConnector
- **Purpose**: Connector class for interacting with Qdrant vector database
- **Attributes**:
  - `url: str` - Qdrant instance URL
  - `api_key: str` - Qdrant API key
  - `collection_name: str` - Name of the Qdrant collection
  - `client: QdrantClient` - Qdrant client instance
- **Methods**:
  - `connect() -> bool` - Test connection to Qdrant instance
  - `connect_with_retry(max_retries: int = 3, delay: float = 1.0) -> bool` - Connection with retry logic
  - `search(query_vector: List[float], top_k: int = 10, metadata_filter: Dict[str, Any] = None) -> List[DocumentChunk]` - Perform similarity search
  - `search_with_text(query_text: str, top_k: int = 10, metadata_filter: Dict[str, Any] = None) -> List[SimilarityScore]` - Perform similarity search using text query
  - `text_to_vector(text: str) -> List[float]` - Convert text to vector embedding
  - `get_chunk_metadata(chunk_ids: List[str]) -> List[Dict[str, Any]]` - Retrieve metadata for specific chunk IDs

### 4. RetrievalValidator
- **Purpose**: Validator class for testing retrieval accuracy, metadata integrity, and consistency
- **Attributes**:
  - `qdrant_connector: QdrantConnector` - Connector instance for Qdrant
  - `connected: bool` - Whether the validator is connected to Qdrant
- **Methods**:
  - `connect() -> bool` - Connect to Qdrant and verify connection
  - `validate_single_query(query_text: str, expected_chunks: Optional[List[str]] = None) -> ValidationResult` - Validate a single query
  - `validate_retrieval_accuracy(queries: List[str]) -> List[ValidationResult]` - Validate retrieval accuracy for a list of queries
  - `validate_consistency(query: str, num_runs: int = None) -> Dict[str, Any]` - Validate consistency of results
  - `run_complete_validation(sample_queries: List[str] = None) -> ValidationSummary` - Run complete validation

## Function Models

### 1. retrieve_information(query: str, top_k: int = 5, threshold: float = 0.7) -> RetrievalResult
- **Purpose**: Function tool for retrieving relevant information from the knowledge base
- **Parameters**:
  - `query: str` - The search query
  - `top_k: int` - Number of top results to return (default: 5)
  - `threshold: float` - Minimum similarity threshold (default: 0.7)
- **Returns**: RetrievalResult containing the retrieved information
- **Decorator**: `@function_tool` - Marks this as an agent tool

## Data Flow

### 1. Query Processing Flow
```
User Query → Agent → retrieve_information tool → QdrantConnector → Search → DocumentChunks → SimilarityScores → RetrievalResult → Agent → Response
```

### 2. Configuration Flow
```
Environment Variables → ValidationConfig → QdrantConnector/RetrievalValidator → Agent Initialization
```

### 3. Response Generation Flow
```
Query → Retrieval → Context Injection → Agent Processing → Grounded Response Generation
```

## Relationships

### 1. RetrievalAugmentedAgent depends on:
- ValidationConfig for configuration
- QdrantConnector for retrieval functionality
- RetrievalResult for structured output
- Agent SDK (or mock) for agent functionality

### 2. QdrantConnector uses:
- DocumentChunk for search results
- SimilarityScore for similarity measurements
- ValidationConfig for connection parameters
- Cohere API for text-to-vector conversion

### 3. RetrievalValidator depends on:
- QdrantConnector for Qdrant operations
- ValidationResult for validation results
- ValidationSummary for validation summaries
- ValidationConfig for configuration