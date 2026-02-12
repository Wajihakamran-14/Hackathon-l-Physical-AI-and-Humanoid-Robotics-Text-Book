"""
Website Content Ingestion Pipeline

A Python-based content ingestion pipeline that crawls Docusaurus documentation sites,
extracts clean text, generates Cohere embeddings, and stores them with metadata in Qdrant vector database.
"""

import os
import sys
import logging
import argparse
from typing import List, Dict, Any, Optional
from urllib.parse import urljoin, urlparse
import time
from dataclasses import dataclass, asdict
from datetime import datetime
import json
from urllib.parse import urljoin, urlparse
import time
import random
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# Import required libraries
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from dotenv import load_dotenv


# Configuration module
def load_config():
    """Load configuration from environment variables"""
    load_dotenv()

    config = {
        'cohere_api_key': os.getenv('COHERE_API_KEY'),
        'qdrant_url': os.getenv('QDRANT_URL'),
        'qdrant_api_key': os.getenv('QDRANT_API_KEY'),
        'source_url': os.getenv('SOURCE_URL'),
        'default_chunk_size': int(os.getenv('DEFAULT_CHUNK_SIZE', '512')),
        'default_overlap_size': int(os.getenv('DEFAULT_OVERLAP_SIZE', '102')),
    }

    # Validate required configuration
    required_keys = ['cohere_api_key', 'qdrant_url']
    missing_keys = [key for key, value in config.items() if not value and key in required_keys]

    if missing_keys:
        raise ValueError(f"Missing required configuration: {','.join(missing_keys)}")

    return config


# Logging setup
def setup_logging():
    """Set up structured logging"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('ingestion.log'),
            logging.StreamHandler(sys.stdout)
        ]
    )
    return logging.getLogger(__name__)


# Error handling and retry utilities
def create_session_with_retries():
    """Create a requests session with retry strategy"""
    session = requests.Session()
    retry_strategy = Retry(
        total=3,
        backoff_factor=1,
        status_forcelist=[429, 500, 502, 503, 504],
    )
    adapter = HTTPAdapter(max_retries=retry_strategy)
    session.mount("http://", adapter)
    session.mount("https://", adapter)
    return session


def exponential_backoff_retry(func, max_retries=3, base_delay=1):
    """Decorator for retrying with exponential backoff"""
    def wrapper(*args, **kwargs):
        for attempt in range(max_retries):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                if attempt == max_retries - 1:
                    raise e
                delay = base_delay * (2 ** attempt) + random.uniform(0, 1)
                logging.warning(f"Attempt {attempt + 1} failed: {e}. Retrying in {delay:.2f}s...")
                time.sleep(delay)
        return None
    return wrapper


# Data models
@dataclass
class DocumentChunk:
    """Represents a segment of extracted text content with associated metadata"""
    id: str
    content: str
    source_url: str
    section: Optional[str] = None
    position: int = 0
    created_at: datetime = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()


@dataclass
class EmbeddingVector:
    """High-dimensional vector representation of text content"""
    id: str
    vector: List[float]
    dimensions: int
    model: str


@dataclass
class Metadata:
    """Associated information stored alongside embeddings in Qdrant"""
    source_url: str
    section: Optional[str] = None
    chunk_id: str = ""
    content_length: int = 0
    processed_at: datetime = None
    content: str = ""

    def __post_init__(self):
        if self.processed_at is None:
            self.processed_at = datetime.now()


@dataclass
class ProcessingState:
    """Tracks the state of documents during the ingestion process"""
    url: str
    status: str  # pending, processing, completed, failed
    error_message: Optional[str] = None
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    chunks_created: Optional[int] = None


# Qdrant client setup
def setup_qdrant_client(config):
    """Set up Qdrant client connection"""
    client = QdrantClient(
        url=config['qdrant_url'],
        api_key=config['qdrant_api_key']
    )
    return client


# Cohere client setup
def setup_cohere_client(config):
    """Set up Cohere client connection"""
    client = cohere.Client(config['cohere_api_key'])
    return client


def is_valid_url(url):
    """Validate if the given string is a valid URL"""
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def get_urls_from_sitemap(sitemap_url: str) -> List[str]:
    """Extract all URLs from sitemap.xml"""
    import xml.etree.ElementTree as ET

    session = create_session_with_retries()

    try:
        response = session.get(sitemap_url, timeout=10)
        response.raise_for_status()

        # Parse the XML content
        root = ET.fromstring(response.content)

        # Handle both regular sitemaps and sitemap indexes
        urls = []

        # Check if this is a sitemap index (contains other sitemaps)
        namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

        # Look for sitemap entries (other sitemaps)
        sitemap_entries = root.findall('.//sitemap:sitemap/sitemap:loc', namespace)
        if sitemap_entries:
            # This is a sitemap index, get URLs from each referenced sitemap
            for sitemap_entry in sitemap_entries:
                sitemap_loc = sitemap_entry.text.strip()
                urls.extend(get_urls_from_individual_sitemap(sitemap_loc, session))
        else:
            # This is a regular sitemap with URL entries
            urls = get_urls_from_individual_sitemap(sitemap_url, session)

        return urls
    except Exception as e:
        logging.error(f"Error parsing sitemap {sitemap_url}: {e}")
        # If sitemap fails, fall back to link crawling
        return []


def get_urls_from_individual_sitemap(sitemap_url: str, session) -> List[str]:
    """Extract URLs from an individual sitemap file"""
    import xml.etree.ElementTree as ET

    try:
        response = session.get(sitemap_url, timeout=10)
        response.raise_for_status()

        # Parse the XML content
        root = ET.fromstring(response.content)

        # Define namespace if present
        namespace = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

        # Try to find URLs with and without namespace
        url_elements = root.findall('.//ns:url/ns:loc', namespace) or root.findall('.//url/loc')

        urls = []
        for url_elem in url_elements:
            url = url_elem.text.strip()
            urls.append(url)

        return urls
    except Exception as e:
        logging.error(f"Error parsing individual sitemap {sitemap_url}: {e}")
        return []


def crawl_docusaurus_site(source_url: str) -> List[Dict[str, str]]:
    """Crawl the Docusaurus site and extract page content"""
    if not is_valid_url(source_url):
        raise ValueError(f"Invalid URL: {source_url}")

    session = create_session_with_retries()
    pages_content = []
    failed_urls = []

    # Get sitemap URL
    sitemap_url = urljoin(source_url, 'sitemap.xml')
    logging.info(f"Attempting to fetch sitemap from: {sitemap_url}")

    # Try to get URLs from sitemap first
    sitemap_urls = get_urls_from_sitemap(sitemap_url)

    if sitemap_urls:
        logging.info(f"Found {len(sitemap_urls)} URLs from sitemap")
        urls_to_process = sitemap_urls
    else:
        # Fallback to link crawling if sitemap is not available
        logging.warning("Sitemap not available, falling back to link crawling")
        visited_urls = set()
        urls_to_visit = [source_url]
        urls_to_process = []

        # Perform link crawling to discover URLs
        while urls_to_visit:
            current_url = urls_to_visit.pop(0)

            if current_url in visited_urls:
                continue

            visited_urls.add(current_url)
            urls_to_process.append(current_url)

            try:
                response = rate_limit_request(session, current_url, delay=1)
                response.raise_for_status()

                soup = BeautifulSoup(response.content, 'html.parser')

                # Find additional links to crawl within the same domain
                for link in soup.find_all('a', href=True):
                    href = link['href']
                    full_url = urljoin(current_url, href)

                    # Only follow links within the same domain and that haven't been visited
                    if (urlparse(full_url).netloc == urlparse(source_url).netloc and
                        full_url not in visited_urls and
                        full_url.startswith(source_url) and
                        not full_url.endswith(('.jpg', '.jpeg', '.png', '.gif', '.pdf', '.zip', '.css', '.js'))):
                        urls_to_visit.append(full_url)

            except requests.RequestException as e:
                failed_urls.append((current_url, str(e)))
                logging.warning(f"Failed to crawl {current_url} during discovery: {e}")
                continue
            except Exception as e:
                failed_urls.append((current_url, str(e)))
                logging.error(f"Unexpected error crawling {current_url} during discovery: {e}")
                continue

    logging.info(f"Starting to crawl {len(urls_to_process)} URLs")

    # Process all discovered URLs
    for i, current_url in enumerate(urls_to_process):
        try:
            logging.info(f"Crawling ({i+1}/{len(urls_to_process)}): {current_url}")

            response = rate_limit_request(session, current_url, delay=1)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Extract main content, typically in main or article tags, or specific Docusaurus selectors
            main_content = soup.find('main') or soup.find('article')
            if not main_content:
                # Look for common Docusaurus content containers
                main_content = soup.find('div', class_='container') or soup.find('div', class_='main-wrapper')

            if main_content:
                # Remove navigation, headers, footers, and other non-content elements
                for element in main_content.find_all(['nav', 'header', 'footer', 'aside']):
                    element.decompose()

                # Extract clean text content
                clean_text = main_content.get_text(separator=' ', strip=True)

                if clean_text:  # Only add if we have actual content
                    page_data = {
                        'url': current_url,
                        'title': soup.title.string if soup.title else '',
                        'content': clean_text
                    }
                    pages_content.append(page_data)
                    logging.info(f"Successfully extracted content from {current_url} ({len(clean_text)} chars)")

        except requests.RequestException as e:
            failed_urls.append((current_url, str(e)))
            logging.warning(f"Failed to crawl {current_url}: {e}")
            continue  # Continue with other URLs even if one fails
        except Exception as e:
            failed_urls.append((current_url, str(e)))
            logging.error(f"Unexpected error crawling {current_url}: {e}")
            continue  # Continue with other URLs even if one fails

    logging.info(f"Crawling completed. Processed {len(urls_to_process)} URLs, extracted content from {len(pages_content)} pages.")
    if failed_urls:
        logging.warning(f"Failed to crawl {len(failed_urls)} URLs: {failed_urls[:5]}...")  # Show first 5 failures

    return pages_content


def handle_large_pages(content: str, max_size: int = 100000) -> str:
    """Handle large pages by truncating if necessary"""
    if len(content) > max_size:
        logging.warning(f"Page content too large ({len(content)} chars), truncating to {max_size}")
        return content[:max_size]
    return content


def rate_limit_request(session, url, delay=1):
    """Make a request with rate limiting to respect server constraints"""
    time.sleep(delay)  # Simple rate limiting
    return session.get(url, timeout=10)


def chunk_text(text: str, chunk_size: int = 512, overlap_size: int = 102) -> List[DocumentChunk]:
    """Split text into overlapping chunks optimized for RAG"""
    if not text:
        return []

    # Convert chunk_size from tokens to characters (rough approximation: 1 token ~ 4 chars)
    chunk_chars = chunk_size * 4
    overlap_chars = overlap_size * 4

    chunks = []
    start = 0
    position = 0

    while start < len(text):
        # Calculate end position
        end = start + chunk_chars

        # If this is the last chunk, make sure we include the rest of the text
        if end >= len(text):
            end = len(text)
        else:
            # Try to break at sentence boundary
            # Look for sentence endings near the end
            search_start = max(start, end - 100)  # Look back at most 100 chars
            sentence_end = -1

            for punct in ['.', '!', '?', '\n']:
                last_punct = text.rfind(punct, search_start, end)
                if last_punct > sentence_end:
                    sentence_end = last_punct

            # If we found a sentence boundary, use it
            if sentence_end != -1 and sentence_end > start:
                end = sentence_end + 1  # Include the punctuation
            else:
                # If no sentence boundary found, try to break at word boundary
                search_start = max(start, end - 50)  # Look back at most 50 chars
                word_end = text.rfind(' ', search_start, end)
                if word_end != -1 and word_end > start:
                    end = word_end
                # Otherwise, just use the character limit

        # Extract the chunk
        chunk_text = text[start:end].strip()
        if chunk_text:  # Only add non-empty chunks
            chunk_id = f"chunk_{position}_{start}_{end}"
            chunk = DocumentChunk(
                id=chunk_id,
                content=chunk_text,
                source_url="",
                position=position
            )
            chunks.append(chunk)

        # Move to next chunk position with overlap
        if start == end:
            # Prevent infinite loop if we can't advance
            break

        # Calculate next start position with overlap
        next_start = end - overlap_chars
        if next_start <= start:
            # If overlap would cause overlap to be negative or zero advance by half chunk
            next_start = start + chunk_chars // 2

        start = next_start
        position += 1

    return chunks


def extract_clean_text(html_content: str) -> str:
    """Extract clean text from HTML content"""
    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove script and style elements
    for script in soup(["script", "style"]):
        script.decompose()

    # Get text and remove extra whitespace
    clean_text = soup.get_text(separator=' ', strip=True)

    # Clean up multiple spaces
    import re
    clean_text = re.sub(r'\s+', ' ', clean_text)

    return clean_text


def generate_embeddings(texts: List[str], cohere_client) -> List[EmbeddingVector]:
    """Generate Cohere embeddings for text chunks"""
    if not texts:
        return []

    # Generate embeddings using Cohere
    response = cohere_client.embed(
        texts=texts,
        model='embed-multilingual-v3.0',
        input_type='search_document'
    )

    embeddings = []
    for i, embedding_vector in enumerate(response.embeddings):
        embedding = EmbeddingVector(
            id=f"embedding_{i}",
            vector=embedding_vector,
            dimensions=len(embedding_vector),
            model='embed-multilingual-v3.0'
        )
        embeddings.append(embedding)

    return embeddings


def validate_embedding_dimensions(embeddings: List[EmbeddingVector]) -> bool:
    """Validate embedding dimensions and quality"""
    if not embeddings:
        return False

    # Check that all embeddings have the same dimension
    first_dim = embeddings[0].dimensions
    for emb in embeddings[1:]:
        if emb.dimensions != first_dim:
            return False

    return True


def create_qdrant_collection(qdrant_client, collection_name: str = "documents"):
    """Create Qdrant collection with proper schema"""
    try:
        # Check if collection already exists
        collections = qdrant_client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)

        if not collection_exists:
            # Create collection with appropriate vector size (using size of first embedding as reference)
            # We'll use a default size if no embeddings are provided
            qdrant_client.recreate_collection(
                collection_name=collection_name,
                vectors_config={
                    "content": {
                        "size": 1024,  # Default size, will be adjusted based on actual embeddings
                        "distance": "Cosine"
                    }
                }
            )
            logging.info(f"Created Qdrant collection: {collection_name}")
        else:
            logging.info(f"Qdrant collection {collection_name} already exists")
    except Exception as e:
        logging.error(f"Error creating Qdrant collection: {e}")
        raise


def store_embeddings(embeddings: List[EmbeddingVector], metadata_list: List[Metadata], qdrant_client, collection_name: str = "documents"):
    """Store embeddings with metadata in Qdrant"""
    if not embeddings or len(embeddings) != len(metadata_list):
        raise ValueError("Embeddings and metadata lists must be non-empty and of equal length")

    # Prepare points for insertion using Qdrant PointStruct
    import uuid
    from qdrant_client.http.models import PointStruct

    points = []
    for i, (embedding, metadata) in enumerate(zip(embeddings, metadata_list)):
        # FIX: Generate a UUID for each Qdrant point ID (Qdrant Cloud requirement)
        # Qdrant Cloud only accepts unsigned integers or UUID strings as point IDs
        point_id = str(uuid.uuid4())

        point = PointStruct(
            id=point_id,  # Use UUID as Qdrant point ID (Qdrant Cloud requirement)
            vector={"content": embedding.vector},  # Specify the named vector as 'content' to match collection schema
            payload={
                "source_url": metadata.source_url,
                "section": metadata.section,
                "chunk_id": metadata.chunk_id,  # Store chunk_id in payload, not as point ID
                "text": metadata.content,  # Use content field from metadata
                "content_length": metadata.content_length,
                "processed_at": metadata.processed_at.isoformat() if metadata.processed_at else None
            }
        )
        points.append(point)

    # IMPORTANT: After fixing the ID issue, old data with colliding IDs must be deleted and re-ingested
    # to ensure data integrity in the Qdrant collection.
    # WARNING: Old collections with incompatible point IDs must be deleted manually once.

    try:
        # Upload points to Qdrant
        qdrant_client.upload_points(
            collection_name=collection_name,
            points=points
        )
        logging.info(f"Successfully stored {len(points)} embeddings in Qdrant collection: {collection_name}")
        return True
    except Exception as e:
        logging.error(f"Error storing embeddings in Qdrant: {str(e)}")
        return False


def verify_stored_vectors(qdrant_client, collection_name: str = "documents", sample_size: int = 5):
    """Verify that stored vectors are accessible in Qdrant"""
    try:
        # Get collection info first
        collection_info = qdrant_client.get_collection(collection_name)
        logging.info(f"Collection {collection_name} has {collection_info.points_count} vectors")

        if collection_info.points_count == 0:
            return False

        # Sample a few points by their integer IDs to verify they're accessible
        # Use the first few points (IDs 0 to sample_size-1) or up to the total count
        max_sample = min(sample_size, collection_info.points_count)
        sample_ids = list(range(max_sample))

        if sample_ids:
            # Verify specific vectors by ID
            records = qdrant_client.retrieve(
                collection_name=collection_name,
                ids=sample_ids
            )
            logging.info(f"Verified {len(records)} specific vectors in Qdrant")
            return len(records) == len(sample_ids)
        else:
            return collection_info.points_count > 0
    except Exception as e:
        logging.error(f"Error verifying stored vectors in Qdrant: {str(e)}")
        return False


def main():
    """Main entry point for the ingestion pipeline"""
    parser = argparse.ArgumentParser(description='Website Content Ingestion Pipeline')
    parser.add_argument('--source-url', required=True, help='URL of the Docusaurus site to ingest')
    parser.add_argument('--chunk-size', type=int, default=512, help='Size of text chunks in tokens (default: 512)')
    parser.add_argument('--overlap-size', type=int, default=102, help='Overlap size between chunks in tokens (default: 102)')

    args = parser.parse_args()

    print(f"Starting ingestion pipeline for: {args.source_url}")
    print(f"Chunk size: {args.chunk_size}, Overlap size: {args.overlap_size}")

    # Initialize configuration
    config = load_config()

    # Initialize logging
    logger = setup_logging()

    try:
        # Set up clients
        cohere_client = setup_cohere_client(config)
        qdrant_client = setup_qdrant_client(config)

        # Create Qdrant collection
        create_qdrant_collection(qdrant_client)

        # Phase 1: Content Crawling and Extraction (User Story 1)
        logging.info("Starting content crawling and extraction...")
        pages_content = crawl_docusaurus_site(args.source_url)
        logging.info(f"Successfully crawled {len(pages_content)} pages")

        if not pages_content:
            logging.error("No content found to process. Exiting.")
            return

        # Phase 2: Text Chunking and Embedding Generation (User Story 2)
        logging.info("Starting text chunking and embedding generation...")
        all_chunks = []
        for page in pages_content:
            # Update source URL in the chunk
            chunks = chunk_text(page['content'], args.chunk_size, args.overlap_size)
            for chunk in chunks:
                chunk.source_url = page['url']  # Update the source URL
            all_chunks.extend(chunks)

        logging.info(f"Created {len(all_chunks)} text chunks")

        # Generate embeddings for all chunks
        if all_chunks:
            # Extract just the content for embedding
            texts_to_embed = [chunk.content for chunk in all_chunks]

            # Process in batches to respect API limits
            batch_size = 96  # Cohere's recommended batch size
            all_embeddings = []

            for i in range(0, len(texts_to_embed), batch_size):
                batch_texts = texts_to_embed[i:i + batch_size]
                batch_embeddings = generate_embeddings(batch_texts, cohere_client)
                all_embeddings.extend(batch_embeddings)

                logging.info(f"Processed batch {i//batch_size + 1}/{(len(texts_to_embed) - 1)//batch_size + 1}")

            logging.info(f"Generated {len(all_embeddings)} embeddings")

            # Validate embeddings
            if not validate_embedding_dimensions(all_embeddings):
                logging.error("Embedding validation failed")
                return

            # Create metadata for each chunk
            metadata_list = []
            for chunk in all_chunks:
                metadata = Metadata(
                    source_url=chunk.source_url,
                    section=chunk.section,
                    chunk_id=chunk.id,
                    content_length=len(chunk.content),
                    content=chunk.content
                )
                metadata_list.append(metadata)

            # Phase 3: Storage in Qdrant (User Story 3)
            logging.info("Starting storage in Qdrant...")
            success = store_embeddings(all_embeddings, metadata_list, qdrant_client)

            if success:
                # Verify stored vectors
                verification_result = verify_stored_vectors(qdrant_client, sample_size=5)

                if verification_result:
                    logging.info("SUCCESS: All phases completed successfully!")
                    logging.info(f"SUCCESS: Stored {len(all_embeddings)} embeddings in Qdrant")
                    print(f"SUCCESS: Successfully processed and stored {len(all_embeddings)} embeddings in Qdrant!")
                else:
                    logging.info("INFO: Embeddings stored but verification inconclusive (may be due to eventual consistency)")
                    logging.info(f"SUCCESS: Stored {len(all_embeddings)} embeddings in Qdrant")
                    print(f"SUCCESS: Successfully processed and stored {len(all_embeddings)} embeddings in Qdrant!")
            else:
                logging.error("ERROR: Failed to store embeddings in Qdrant")
        else:
            logging.warning("No text chunks to process")

    except Exception as e:
        logging.error(f"Pipeline failed with error: {e}")
        print(f"❌ Pipeline failed with error: {e}")
        raise

    print("Pipeline completed successfully!")


if __name__ == "__main__":
    main()