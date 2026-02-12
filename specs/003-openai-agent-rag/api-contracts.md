# API Contracts: OpenAI Agent RAG Implementation

## Public API Endpoints

### 1. RetrievalAugmentedAgent Class Interface

#### Constructor
```
RetrievalAugmentedAgent.__init__(collection_name: str = None, similarity_threshold: float = 0.7)
```
- **Purpose**: Initialize the retrieval augmented agent using OpenAI Agents SDK
- **Parameters**:
  - `collection_name: str` (optional) - Name of the Qdrant collection to use
  - `similarity_threshold: float` (optional, default: 0.7) - Minimum similarity score for relevant results
- **Returns**: `RetrievalAugmentedAgent` instance
- **Exceptions**: May raise exceptions during initialization if Qdrant connection fails

#### Synchronous Query Method
```
RetrievalAugmentedAgent.query(user_question: str) -> str
```
- **Purpose**: Process a user question using the agent with retrieval augmentation
- **Parameters**:
  - `user_question: str` - The question from the user
- **Returns**: `str` - The answer based on retrieved content
- **Behavior**:
  - Uses OpenAI Agents SDK if available
  - Falls back to basic retrieval if agent SDK is not available
  - Handles exceptions and falls back to basic retrieval
- **Errors**: Logs errors and falls back to basic retrieval

#### Asynchronous Query Method
```
RetrievalAugmentedAgent.query_async(user_question: str) -> str
```
- **Purpose**: Process a user question using the agent with retrieval augmentation (async version)
- **Parameters**:
  - `user_question: str` - The question from the user
- **Returns**: `str` - The answer based on retrieved content
- **Behavior**:
  - Uses OpenAI Agents SDK asynchronously if available
  - Falls back to basic retrieval if agent SDK is not available
  - Handles exceptions and falls back to basic retrieval
- **Errors**: Logs errors and falls back to basic retrieval

### 2. Agent Tool Interface

#### retrieve_information Function Tool
```
retrieve_information(query: str, top_k: int = 5, threshold: float = 0.7) -> RetrievalResult
```
- **Purpose**: Retrieve relevant information from the knowledge base based on the query
- **Parameters**:
  - `query: str` - The search query
  - `top_k: int` (optional, default: 5) - Number of top results to return
  - `threshold: float` (optional, default: 0.7) - Minimum similarity threshold
- **Returns**: `RetrievalResult` - Structure containing the retrieved information
- **Behavior**:
  - Connects to Qdrant using ValidationConfig
  - Performs similarity search using text query
  - Filters results based on similarity threshold
  - Returns structured results with content and metadata
- **Errors**: Returns RetrievalResult with error information if connection or search fails

## Data Transfer Objects (DTOs)

### 1. RetrievalResult Schema
```
{
  "query": "string",
  "results": [
    {
      "chunk_id": "string",
      "content": "string",
      "similarity_score": "number",
      "source_url": "string"
    }
  ],
  "total_found": "integer",
  "relevant_count": "integer"
}
```
- **Purpose**: Structure for retrieval results returned by the agent tool
- **Fields**:
  - `query: string` - The original search query
  - `results: array` - List of retrieved results with content and metadata
  - `total_found: integer` - Total number of results found
  - `relevant_count: integer` - Number of results meeting relevance threshold

## Internal API Interfaces

### 1. Configuration Interface

#### ValidationConfig
- **Purpose**: Configuration for retrieval validation
- **Properties**:
  - `qdrant_url: string` - Qdrant instance URL
  - `qdrant_api_key: string` - Qdrant API key (if required)
  - `collection_name: string` - Name of the Qdrant collection to use
  - `similarity_threshold: number` - Minimum similarity threshold
  - `num_test_runs: integer` - Number of test runs for consistency validation
  - `validate_metadata: boolean` - Whether to validate metadata
  - `max_memory_usage: integer` - Maximum memory usage in MB
  - `max_query_time: number` - Maximum query time in seconds
  - `default_sample_queries: string[]` - Default queries for validation

### 2. QdrantConnector Interface

#### Search Method
```
QdrantConnector.search(query_vector: number[], top_k: integer = 10, metadata_filter: object = null) -> DocumentChunk[]
```
- **Purpose**: Perform similarity search in Qdrant collection
- **Parameters**:
  - `query_vector: number[]` - Vector to search for similar items
  - `top_k: integer` (optional, default: 10) - Number of top results to return
  - `metadata_filter: object` (optional, default: null) - Filter for metadata fields
- **Returns**: `DocumentChunk[]` - List of document chunks matching the search
- **Errors**: Returns empty array if search fails

#### Search with Text Method
```
QdrantConnector.search_with_text(query_text: string, top_k: integer = 10, metadata_filter: object = null) -> SimilarityScore[]
```
- **Purpose**: Perform similarity search using text query
- **Parameters**:
  - `query_text: string` - Text to convert to vector and search for
  - `top_k: integer` (optional, default: 10) - Number of top results to return
  - `metadata_filter: object` (optional, default: null) - Filter for metadata fields
- **Returns**: `SimilarityScore[]` - List of similarity scores for matching chunks
- **Errors**: Returns empty array if search fails

#### Text to Vector Method
```
QdrantConnector.text_to_vector(text: string) -> number[]
```
- **Purpose**: Convert text to vector embedding using Cohere
- **Parameters**:
  - `text: string` - Text to convert to vector
- **Returns**: `number[]` - Vector embedding of the text
- **Errors**: Returns empty array if conversion fails

## CLI Interface

### Command Line Arguments
```
python agent.py [--query QUERY] [--collection COLLECTION] [--threshold THRESHOLD]
```

#### Arguments:
- `--query` (string): Single query to process
- `--collection` (string): Qdrant collection name (default: from env var QDRANT_COLLECTION)
- `--threshold` (float): Similarity threshold for relevant results (default: 0.7)

#### Behavior:
- If `--query` is provided, processes single query and exits
- If no `--query`, enters interactive mode
- Uses collection and threshold parameters for agent configuration
- Provides feedback about OpenAI Agents SDK availability

## Error Handling Contracts

### 1. Connection Error Handling
- **Contract**: If Qdrant connection fails, return appropriate error message
- **Implementation**: Uses `connect_with_retry()` method with configurable retries
- **Fallback**: Returns error result in RetrievalResult when connection fails

### 2. Agent SDK Unavailability Handling
- **Contract**: If OpenAI Agents SDK is not available, use basic retrieval functionality
- **Implementation**: Conditional imports with mock classes as fallback
- **Fallback**: Uses `_basic_retrieve_and_answer()` method

### 3. Query Processing Error Handling
- **Contract**: If query processing fails, log error and fall back to basic retrieval
- **Implementation**: Try-catch blocks in query methods with fallback calls
- **Fallback**: Uses `_basic_retrieve_and_answer()` method

## Performance Contracts

### 1. Response Time
- **Contract**: Agent responses should be returned within reasonable time
- **Implementation**: Configurable `max_query_time` in ValidationConfig
- **Default**: No explicit timeout, but depends on Qdrant and OpenAI API response times

### 2. Memory Usage
- **Contract**: Agent should operate within configurable memory limits
- **Implementation**: Configurable `max_memory_usage` in ValidationConfig
- **Default**: 50 MB (as defined in ValidationConfig)

## Compatibility Contracts

### 1. OpenAI Agents SDK Compatibility
- **Contract**: System should work with OpenAI Agents SDK when available
- **Implementation**: Conditional imports and proper agent initialization
- **Fallback**: Basic retrieval functionality when SDK not available

### 2. Qdrant Client Compatibility
- **Contract**: System should work with different versions of Qdrant client
- **Implementation**: Version-aware method selection in QdrantConnector.search()
- **Fallback**: Checks for available methods and adapts accordingly

## Versioning Strategy

### 1. Backward Compatibility
- **Contract**: API methods should maintain backward compatibility where possible
- **Implementation**: Maintains same method signatures and parameter defaults
- **Approach**: Add new parameters as optional with defaults

### 2. Configuration Evolution
- **Contract**: Configuration should be extensible without breaking changes
- **Implementation**: Uses environment variables with sensible defaults
- **Approach**: New configuration options added as optional with defaults

## Idempotency and Consistency

### 1. Query Repetition
- **Contract**: Same query should produce consistent results when possible
- **Implementation**: Uses vector embeddings which should be deterministic
- **Note**: May vary based on Qdrant index updates or API changes

### 2. State Management
- **Contract**: Agent should not maintain persistent state between queries
- **Implementation**: Each query is processed independently
- **Exception**: Configuration is maintained per agent instance

## Security Considerations

### 1. Input Validation
- **Contract**: Query inputs should be processed safely without injection
- **Implementation**: Uses text-to-vector conversion which should sanitize inputs
- **Note**: Relies on underlying libraries for input sanitization

### 2. API Key Handling
- **Contract**: API keys should be loaded securely from environment
- **Implementation**: Uses environment variables via ValidationConfig
- **Approach**: Never hardcodes API keys in source code