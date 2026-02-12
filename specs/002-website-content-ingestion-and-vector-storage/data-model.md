# Data Model: Website Content Ingestion and Vector Storage

## Document Chunk
**Description**: Represents a segment of extracted text content with associated metadata
- **id**: Unique identifier for the chunk (string, required)
- **content**: The text content of the chunk (string, required)
- **source_url**: URL of the original document (string, required)
- **section**: Section or heading of the document where the chunk originated (string, optional)
- **position**: Position of the chunk within the document (integer, required)
- **created_at**: Timestamp when the chunk was created (datetime, required)

## Embedding Vector
**Description**: High-dimensional vector representation of text content
- **id**: Unique identifier that matches the document chunk ID (string, required)
- **vector**: Array of floating point numbers representing the embedding (array of floats, required)
- **dimensions**: Number of dimensions in the embedding (integer, required)
- **model**: Name of the model used to generate the embedding (string, required)

## Metadata
**Description**: Associated information stored alongside embeddings in Qdrant
- **source_url**: URL of the original document (string, required)
- **section**: Section or heading of the document (string, optional)
- **chunk_id**: Identifier of the source chunk (string, required)
- **content_length**: Length of the original content (integer, required)
- **processed_at**: Timestamp when the content was processed (datetime, required)

## Qdrant Point
**Description**: Complete data structure stored in Qdrant vector database
- **id**: Unique identifier for the point (string, required)
- **vector**: The embedding vector (array of floats, required)
- **payload**: Dictionary containing metadata (object, required)
  - **source_url**: URL of the original document
  - **section**: Document section
  - **chunk_id**: Source chunk identifier
  - **content_length**: Length of original content
  - **processed_at**: Processing timestamp

## Processing State
**Description**: Tracks the state of documents during the ingestion process
- **url**: URL of the document being processed (string, required)
- **status**: Current status of processing (enum: pending, processing, completed, failed, required)
- **error_message**: Error message if processing failed (string, optional)
- **started_at**: Timestamp when processing started (datetime, optional)
- **completed_at**: Timestamp when processing completed (datetime, optional)
- **chunks_created**: Number of chunks created from this document (integer, optional)