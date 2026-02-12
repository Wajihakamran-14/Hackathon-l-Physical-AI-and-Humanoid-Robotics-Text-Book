# Data Model: Retrieval Pipeline Validation

## Document Chunk
- **id**: string - Unique identifier for the document chunk
- **content**: string - The actual text content of the document chunk
- **url**: string - Source URL where the original document is located
- **section**: string - Section identifier within the source document
- **chunk_id**: string - Unique identifier for this specific chunk
- **vector**: array[float] - Vector embedding representation of the content
- **metadata**: dict - Additional metadata associated with the chunk

## Query Vector
- **id**: string - Unique identifier for the query
- **content**: string - Original query text
- **vector**: array[float] - Vector embedding representation of the query
- **expected_results**: array[string] - Expected relevant chunk IDs (optional, for validation)

## Similarity Score
- **chunk_id**: string - ID of the document chunk being scored
- **score**: float - Cosine similarity score between query and chunk
- **rank**: integer - Rank position in the result set

## Validation Result
- **query_id**: string - ID of the query being validated
- **query_content**: string - Original query text
- **retrieved_chunks**: array[Similarity Score] - Chunks returned by retrieval
- **metadata_valid**: boolean - Whether metadata integrity was validated
- **relevance_threshold_met**: boolean - Whether similarity scores meet threshold (>0.7)
- **validation_passed**: boolean - Overall validation result
- **timestamp**: datetime - When validation was performed
- **execution_time**: float - Time taken to execute the query

## Validation Summary
- **total_queries**: integer - Number of queries executed
- **queries_passed**: integer - Number of queries that passed validation
- **relevance_accuracy**: float - Percentage of queries meeting relevance threshold
- **metadata_accuracy**: float - Percentage of chunks with valid metadata
- **consistency_score**: float - Measure of result consistency across multiple runs
- **start_time**: datetime - When validation started
- **end_time**: datetime - When validation ended
- **execution_time**: float - Total execution time