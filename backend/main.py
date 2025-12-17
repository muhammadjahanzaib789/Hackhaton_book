"""
Cohere Embedding Pipeline - Single-File Implementation
Crawls Docusaurus site, generates Cohere embeddings, stores in Qdrant.

Target: https://hackhaton-book-wqyh.vercel.app/
Collection: rag_embedding (1024D, cosine similarity)
"""

import os
import sys
import time
import logging
import hashlib
import re
from datetime import datetime
from typing import List, Dict, Any, Optional
from xml.etree import ElementTree
from urllib.parse import urljoin, urlparse

import requests
from bs4 import BeautifulSoup
import tiktoken
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from dotenv import load_dotenv
from tqdm import tqdm


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("embedding_pipeline.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def get_all_urls(base_url: str) -> List[str]:
    """
    Fetch all documentation URLs from the target Docusaurus site.

    Tries sitemap.xml first, falls back to recursive crawling if not available.

    Args:
        base_url: Root URL of the Docusaurus site

    Returns:
        List of absolute documentation URLs

    Raises:
        Exception: If both sitemap and crawling fail
    """
    # Try sitemap.xml first
    sitemap_url = urljoin(base_url, "/sitemap.xml")
    logger.info(f"Fetching sitemap from {sitemap_url}")

    try:
        response = requests.get(sitemap_url, timeout=10)
        response.raise_for_status()

        # Parse XML sitemap
        root = ElementTree.fromstring(response.content)

        # Extract all <loc> URLs
        namespace = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"}
        urls = []
        for loc in root.findall(".//ns:loc", namespace):
            url = loc.text.strip()
            # Filter for /docs/ paths only
            if "/docs/" in url:
                # Rewrite URL to use TARGET_URL base instead of sitemap base
                # This handles cases where sitemap has wrong base URL
                parsed_url = urlparse(url)
                parsed_base = urlparse(base_url)
                rewritten_url = f"{parsed_base.scheme}://{parsed_base.netloc}{parsed_url.path}"
                urls.append(rewritten_url)
                logger.debug(f"Rewrote {url} -> {rewritten_url}")

        if urls:
            logger.info(f"Found {len(urls)} URLs from sitemap (rewritten to {urlparse(base_url).netloc})")
            return sorted(set(urls))  # Deduplicate and sort
        else:
            logger.warning("Sitemap found but no /docs/ URLs extracted")
    except Exception as e:
        logger.warning(f"Sitemap fetch failed: {e}, falling back to recursive crawl")

    # Fallback: Recursive crawler
    logger.info(f"Starting recursive crawl from {base_url}/docs")
    visited = set()
    to_visit = [urljoin(base_url, "/docs/")]
    docs_urls = []
    max_depth = 10
    depth_map = {to_visit[0]: 0}

    while to_visit:
        current_url = to_visit.pop(0)
        current_depth = depth_map.get(current_url, 0)

        if current_url in visited or current_depth > max_depth:
            continue

        visited.add(current_url)

        try:
            # Retry logic with exponential backoff
            for attempt in range(3):
                try:
                    response = requests.get(current_url, timeout=10)
                    response.raise_for_status()
                    break
                except (requests.Timeout, requests.ConnectionError) as e:
                    if attempt < 2:
                        wait_time = 2 ** attempt  # 1s, 2s, 4s
                        logger.warning(f"Retry {attempt + 1}/3 for {current_url} after {wait_time}s")
                        time.sleep(wait_time)
                    else:
                        raise

            # Add to results if it's a docs page
            if "/docs/" in current_url:
                docs_urls.append(current_url)

            # Find all links on the page
            soup = BeautifulSoup(response.content, "html.parser")
            for link in soup.find_all("a", href=True):
                href = link["href"]
                absolute_url = urljoin(current_url, href)

                # Only follow links within the same domain and /docs/ path
                if urlparse(absolute_url).netloc == urlparse(base_url).netloc:
                    if "/docs/" in absolute_url and absolute_url not in visited:
                        to_visit.append(absolute_url)
                        depth_map[absolute_url] = current_depth + 1

        except Exception as e:
            logger.error(f"Failed to crawl {current_url}: {e}")
            continue

    if not docs_urls:
        raise Exception("Both sitemap and recursive crawl failed to find any /docs/ URLs")

    logger.info(f"Recursive crawl found {len(docs_urls)} URLs")
    return sorted(set(docs_urls))


def extract_text_from_url(url: str) -> Dict[str, Any]:
    """
    Clean HTML and extract main article content from a Docusaurus page.

    Args:
        url: URL to extract text from

    Returns:
        Dict with url, title, text, content_hash, extracted_at

    Raises:
        Exception: If extraction fails
    """
    # Fetch HTML with User-Agent header
    headers = {"User-Agent": "CoherePipelineBot/1.0"}
    try:
        response = requests.get(url, headers=headers, timeout=10)
        response.raise_for_status()
        html_content = response.content.decode("utf-8", errors="replace")
    except requests.HTTPError as e:
        if e.response.status_code in [404, 500]:
            logger.error(f"HTTP {e.response.status_code} for {url}")
            raise Exception(f"HTTP {e.response.status_code} error for {url}")
        raise
    except Exception as e:
        logger.error(f"Failed to fetch {url}: {e}")
        raise

    # Parse HTML with BeautifulSoup
    soup = BeautifulSoup(html_content, "html.parser")

    # Extract title (try <h1> first, fallback to <title>)
    title = ""
    h1_tag = soup.find("h1")
    if h1_tag:
        title = h1_tag.get_text().strip()
    else:
        title_tag = soup.find("title")
        if title_tag:
            title = title_tag.get_text().strip()

    # Preserve code blocks by wrapping with ``` markers
    for code_block in soup.find_all("pre"):
        code_tag = code_block.find("code")
        if code_tag:
            code_text = code_tag.get_text()
            # Wrap code with markdown fences
            code_block.string = f"```\n{code_text}\n```"

    # Find main article content (try <article> or .markdown selector)
    main_content = soup.find("article")
    if not main_content:
        main_content = soup.find(class_=re.compile(r"markdown"))
    if not main_content:
        main_content = soup  # Fallback to entire page

    # Remove navigation, sidebars, footers
    for unwanted in main_content.find_all(["nav", "aside", "footer"]):
        unwanted.decompose()
    for unwanted in main_content.find_all(class_=re.compile(r"(sidebar|navbar|nav|footer|toc)")):
        unwanted.decompose()

    # Extract clean text with paragraph separation
    text = main_content.get_text(separator="\n\n").strip()

    # Remove excessive whitespace
    text = re.sub(r"\n{3,}", "\n\n", text)
    text = re.sub(r"[ \t]+", " ", text)

    # Generate MD5 content hash
    content_hash = hashlib.md5(text.encode()).hexdigest()

    # Return extracted data
    return {
        "url": url,
        "title": title,
        "text": text,
        "content_hash": content_hash,
        "extracted_at": datetime.utcnow().isoformat() + "Z"
    }


def chunk_text(
    text: str,
    chunk_size: int = 512,
    overlap: int = 50
) -> List[Dict[str, Any]]:
    """
    Split text into semantically meaningful chunks with overlap.

    Args:
        text: Full page text to chunk
        chunk_size: Target chunk size in tokens (default 512)
        overlap: Overlap size in tokens (default 50)

    Returns:
        List of dicts with text, chunk_index, token_count

    Raises:
        Exception: If chunking fails
    """
    # Initialize tiktoken encoder
    encoder = tiktoken.get_encoding("cl100k_base")

    def count_tokens(s: str) -> int:
        return len(encoder.encode(s))

    def split_large_paragraph(para: str) -> List[str]:
        """Recursively split large paragraphs into sentences."""
        if count_tokens(para) <= chunk_size:
            return [para]

        # Check if it's a code block (preserve intact)
        if para.strip().startswith("```") and para.strip().endswith("```"):
            return [para]  # Keep code blocks whole

        # Split by sentences using regex
        sentences = re.split(r"(?<=[.!?])\s+", para)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            test_chunk = current_chunk + " " + sentence if current_chunk else sentence
            if count_tokens(test_chunk) <= chunk_size:
                current_chunk = test_chunk
            else:
                if current_chunk:
                    chunks.append(current_chunk)
                current_chunk = sentence

        if current_chunk:
            chunks.append(current_chunk)

        return chunks

    # Split text by paragraphs (double newline)
    paragraphs = text.split("\n\n")
    chunks = []
    current_chunk = ""
    current_tokens = 0
    chunk_index = 0

    for para in paragraphs:
        para = para.strip()
        if not para:
            continue

        # Handle large paragraphs recursively
        if count_tokens(para) > chunk_size:
            sub_chunks = split_large_paragraph(para)
            for sub in sub_chunks:
                sub_tokens = count_tokens(sub)

                # If current chunk + sub exceeds size, save current
                if current_tokens + sub_tokens > chunk_size and current_chunk:
                    chunks.append({
                        "text": current_chunk.strip(),
                        "chunk_index": chunk_index,
                        "token_count": current_tokens
                    })
                    chunk_index += 1

                    # Start new chunk with overlap from previous
                    overlap_text = " ".join(current_chunk.split()[-overlap:])
                    current_chunk = overlap_text + " " + sub
                    current_tokens = count_tokens(current_chunk)
                else:
                    current_chunk = (current_chunk + "\n\n" + sub) if current_chunk else sub
                    current_tokens = count_tokens(current_chunk)
        else:
            para_tokens = count_tokens(para)

            # If adding this paragraph exceeds chunk_size, save current chunk
            if current_tokens + para_tokens > chunk_size and current_chunk:
                chunks.append({
                    "text": current_chunk.strip(),
                    "chunk_index": chunk_index,
                    "token_count": current_tokens
                })
                chunk_index += 1

                # Start new chunk with overlap
                overlap_text = " ".join(current_chunk.split()[-overlap:])
                current_chunk = overlap_text + "\n\n" + para
                current_tokens = count_tokens(current_chunk)
            else:
                current_chunk = (current_chunk + "\n\n" + para) if current_chunk else para
                current_tokens = count_tokens(current_chunk)

    # Add final chunk
    if current_chunk:
        chunks.append({
            "text": current_chunk.strip(),
            "chunk_index": chunk_index,
            "token_count": current_tokens
        })

    logger.debug(f"Created {len(chunks)} chunks from text with {count_tokens(text)} tokens")
    return chunks


def embed(texts: List[str]) -> List[List[float]]:
    """
    Generate 1024-dimensional embeddings using Cohere embed-english-v3.0 model.

    Args:
        texts: List of text chunks to embed (max 96 per batch)

    Returns:
        List of 1024-dimensional embedding vectors

    Raises:
        Exception: If embedding generation fails
    """
    # Initialize Cohere client
    api_key = os.getenv("COHERE_API_KEY")
    if not api_key:
        raise Exception("COHERE_API_KEY environment variable not set")

    co = cohere.ClientV2(api_key=api_key)
    model = os.getenv("COHERE_MODEL", "embed-english-v3.0")
    batch_size = int(os.getenv("COHERE_BATCH_SIZE", "96"))

    all_embeddings = []

    # Process texts in batches of 96
    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]
        logger.debug(f"Embedding batch {i // batch_size + 1} ({len(batch)} texts)")

        max_retries = 5
        for attempt in range(max_retries):
            try:
                # Call Cohere embed API
                response = co.embed(
                    texts=batch,
                    model=model,
                    input_type="search_document",
                    embedding_types=["float"]
                )

                # Extract embeddings from response
                embeddings = response.embeddings.float_

                # Validate dimensions (must be 1024)
                for idx, embedding in enumerate(embeddings):
                    if len(embedding) != 1024:
                        logger.error(f"Invalid embedding dimension: {len(embedding)} (expected 1024)")
                        raise Exception(f"Embedding dimension mismatch: got {len(embedding)}, expected 1024")

                all_embeddings.extend(embeddings)
                logger.debug(f"Successfully embedded batch with {len(embeddings)} vectors")
                break  # Success, exit retry loop

            except Exception as e:
                error_str = str(e)

                # Check if it's a rate limit error (429)
                if "429" in error_str or "rate" in error_str.lower():
                    if attempt < max_retries - 1:
                        # Exponential backoff: 5s, 10s, 20s, 40s, 80s
                        wait_time = 5 * (2 ** attempt)
                        logger.warning(f"Rate limit hit (429), retrying in {wait_time}s (attempt {attempt + 1}/{max_retries})")
                        time.sleep(wait_time)
                        continue
                    else:
                        logger.error(f"Rate limit exceeded after {max_retries} retries")
                        raise Exception(f"Cohere rate limit exceeded: {e}")
                else:
                    # Other errors, re-raise immediately
                    logger.error(f"Embedding failed: {e}")
                    raise

    logger.info(f"Successfully generated {len(all_embeddings)} embeddings")
    return all_embeddings


def create_collection(collection_name: str = "rag_embedding") -> None:
    """
    Initialize Qdrant collection with proper vector configuration.

    Args:
        collection_name: Name of the Qdrant collection (default "rag_embedding")

    Returns:
        None (side effect: collection created in Qdrant)

    Raises:
        Exception: If collection creation fails
    """
    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise Exception("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    try:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant client: {e}")
        raise Exception(f"Qdrant authentication failed: {e}")

    # Check if collection already exists
    try:
        collections = client.get_collections()
        existing_names = [col.name for col in collections.collections]

        if collection_name in existing_names:
            logger.info(f"Collection '{collection_name}' already exists, skipping creation")
            return
    except Exception as e:
        logger.error(f"Failed to list collections: {e}")
        raise

    # Create collection with 1024D vectors and cosine similarity
    try:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1024,
                distance=Distance.COSINE
            )
        )
        logger.info(f"Successfully created collection '{collection_name}' (1024D, cosine similarity)")
    except Exception as e:
        logger.error(f"Failed to create collection: {e}")
        raise


def save_chunk_to_qdrant(
    chunk_data: Dict[str, Any],
    embedding: List[float],
    collection_name: str = "rag_embedding"
) -> None:
    """
    Upsert a single chunk's embedding and metadata to Qdrant.

    Args:
        chunk_data: Dict with url, title, text, chunk_index, token_count, content_hash
        embedding: 1024-dimensional vector
        collection_name: Target Qdrant collection (default "rag_embedding")

    Returns:
        None (side effect: vector stored in Qdrant)

    Raises:
        Exception: If upsert fails
    """
    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise Exception("QDRANT_URL and QDRANT_API_KEY must be set")

    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    # Generate deterministic point ID using MD5 hash
    point_id_str = f"{chunk_data['url']}_{chunk_data['chunk_index']}"
    point_id_hash = hashlib.md5(point_id_str.encode()).hexdigest()
    # Convert hex to integer for Qdrant (first 16 hex chars = 64 bits)
    point_id = int(point_id_hash[:16], 16)

    # Construct payload with metadata
    payload = {
        "url": chunk_data["url"],
        "title": chunk_data["title"],
        "text": chunk_data["text"],
        "chunk_index": chunk_data["chunk_index"],
        "token_count": chunk_data["token_count"],
        "content_hash": chunk_data["content_hash"],
        "indexed_at": datetime.utcnow().isoformat() + "Z"
    }

    # Create PointStruct
    point = PointStruct(
        id=point_id,
        vector=embedding,
        payload=payload
    )

    # Upsert with retry logic
    max_retries = 3
    for attempt in range(max_retries):
        try:
            client.upsert(
                collection_name=collection_name,
                points=[point]
            )
            logger.debug(f"Upserted point {point_id} to '{collection_name}'")
            return  # Success

        except Exception as e:
            if attempt < max_retries - 1:
                wait_time = 2 ** attempt  # 1s, 2s, 4s
                logger.warning(f"Upsert failed (attempt {attempt + 1}/{max_retries}), retrying in {wait_time}s: {e}")
                time.sleep(wait_time)
            else:
                logger.error(f"Upsert failed after {max_retries} attempts: {e}")
                raise Exception(f"Failed to upsert to Qdrant: {e}")


def main() -> None:
    """
    Orchestrate the entire embedding pipeline.

    Pipeline flow:
    1. Load and validate configuration
    2. Initialize clients and create collection
    3. Fetch all URLs from target site
    4. For each URL: extract text → chunk → embed → store
    5. Display summary statistics

    Returns:
        None (side effects: Qdrant populated, logs written)

    Exit Codes:
        0: Success
        1: Configuration error
        2: Runtime error
    """
    start_time = time.time()
    logger.info("=" * 80)
    logger.info("Cohere Embedding Pipeline - Starting")
    logger.info("=" * 80)

    # Load environment variables
    load_dotenv()

    # Validate required configuration
    required_vars = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY", "TARGET_URL"]
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        logger.error(f"Missing required environment variables: {', '.join(missing_vars)}")
        logger.error("Please check your .env file")
        sys.exit(1)

    target_url = os.getenv("TARGET_URL")
    collection_name = "rag_embedding"
    crawl_delay = float(os.getenv("CRAWL_DELAY_SECONDS", "1.0"))

    logger.info(f"Target URL: {target_url}")
    logger.info(f"Collection: {collection_name}")
    logger.info(f"Crawl delay: {crawl_delay}s")

    # Initialize Qdrant collection
    try:
        logger.info("Creating Qdrant collection (if not exists)...")
        create_collection(collection_name)
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant collection: {e}")
        sys.exit(2)

    # Initialize Qdrant client for incremental update checks
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Fetch all URLs
    try:
        logger.info("Fetching all documentation URLs...")
        urls = get_all_urls(target_url)
        logger.info(f"Found {len(urls)} URLs to process")
    except Exception as e:
        logger.error(f"Failed to fetch URLs: {e}")
        sys.exit(2)

    # Statistics tracking
    total_urls = len(urls)
    total_chunks = 0
    failed_urls = 0
    skipped_urls = 0

    # Main pipeline loop with progress bar
    for url_idx, url in enumerate(tqdm(urls, desc="Processing URLs", unit="page")):
        try:
            logger.info(f"[{url_idx + 1}/{total_urls}] Processing: {url}")

            # Extract text from URL
            page_data = extract_text_from_url(url)

            # Incremental update: check if content hash already exists
            try:
                # Query Qdrant for existing chunks with this URL
                search_result = qdrant_client.scroll(
                    collection_name=collection_name,
                    scroll_filter={
                        "must": [
                            {"key": "url", "match": {"value": url}},
                            {"key": "content_hash", "match": {"value": page_data["content_hash"]}}
                        ]
                    },
                    limit=1
                )

                if search_result[0]:  # Non-empty result means content unchanged
                    logger.info(f"Content unchanged for {url}, skipping")
                    skipped_urls += 1
                    time.sleep(crawl_delay)
                    continue
            except Exception as e:
                logger.warning(f"Could not check for existing content, proceeding: {e}")

            # Chunk text
            chunks = chunk_text(
                page_data["text"],
                chunk_size=int(os.getenv("CHUNK_SIZE", "512")),
                overlap=int(os.getenv("CHUNK_OVERLAP", "50"))
            )
            logger.info(f"Created {len(chunks)} chunks")

            # Generate embeddings for all chunks
            chunk_texts = [chunk["text"] for chunk in chunks]
            embeddings = embed(chunk_texts)

            # Save each chunk to Qdrant
            for chunk, embedding in zip(chunks, embeddings):
                chunk_data = {
                    "url": page_data["url"],
                    "title": page_data["title"],
                    "text": chunk["text"],
                    "chunk_index": chunk["chunk_index"],
                    "token_count": chunk["token_count"],
                    "content_hash": page_data["content_hash"]
                }
                save_chunk_to_qdrant(chunk_data, embedding, collection_name)

            total_chunks += len(chunks)
            logger.info(f"Successfully indexed {len(chunks)} chunks from {url}")

            # Polite crawling delay
            time.sleep(crawl_delay)

        except KeyboardInterrupt:
            logger.info("\nPipeline interrupted by user")
            logger.info(f"Progress: {url_idx}/{total_urls} URLs processed")
            logger.info(f"Total chunks indexed: {total_chunks}")
            sys.exit(0)

        except Exception as e:
            logger.error(f"Failed to process {url}: {e}")
            failed_urls += 1
            continue

    # Summary statistics
    elapsed_time = time.time() - start_time
    logger.info("=" * 80)
    logger.info("Pipeline Complete - Summary")
    logger.info("=" * 80)
    logger.info(f"Total URLs processed: {total_urls}")
    logger.info(f"Total chunks indexed: {total_chunks}")
    logger.info(f"Skipped (unchanged): {skipped_urls}")
    logger.info(f"Failed URLs: {failed_urls}")
    logger.info(f"Elapsed time: {elapsed_time:.1f}s ({elapsed_time / 60:.1f}m)")
    logger.info(f"Average: {elapsed_time / total_urls:.1f}s per URL")
    logger.info("=" * 80)

    if failed_urls > 0:
        logger.warning(f"{failed_urls} URLs failed - check logs for details")

    logger.info("Pipeline finished successfully")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Pipeline interrupted by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(2)
