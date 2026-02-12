# Quickstart Guide: Website Content Ingestion and Vector Storage

## Prerequisites

- Python 3.11 or higher
- uv package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. **Create the backend directory:**
   ```bash
   mkdir backend
   cd backend
   ```

2. **Initialize the Python project with uv:**
   ```bash
   uv init
   ```

3. **Create the main.py file with the ingestion logic:**
   ```bash
   touch main.py
   ```

4. **Set up environment variables:**
   ```bash
   # Create .env.example file
   echo "COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   SOURCE_URL=your_docusaurus_site_url_here" > .env.example
   ```

5. **Install dependencies:**
   ```bash
   uv add requests beautifulsoup4 cohere qdrant-client python-dotenv
   ```

## Configuration

1. **Copy the example environment file:**
   ```bash
   cp .env.example .env
   ```

2. **Update .env with your actual API keys and URLs:**
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_URL`: Your Qdrant Cloud instance URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `SOURCE_URL`: The Docusaurus site URL to ingest

## Running the Ingestion Pipeline

1. **Execute the main script:**
   ```bash
   python main.py
   ```

2. **Or run with uv:**
   ```bash
   uv run main.py
   ```

## Expected Output

The ingestion pipeline will:
1. Crawl all pages from the specified Docusaurus site
2. Extract clean text content from each page
3. Generate embeddings using Cohere
4. Store embeddings with metadata in Qdrant
5. Provide progress updates during processing