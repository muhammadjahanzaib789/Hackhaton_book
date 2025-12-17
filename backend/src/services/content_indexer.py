"""
Content indexing service for parsing and chunking book content.
Implements recursive chunking with 400-token chunks and 80-token overlap.
"""

import logging
import re
from pathlib import Path
from typing import List, Dict, Any, Optional
from uuid import uuid4
import tiktoken

import frontmatter
from markdown import markdown
from bs4 import BeautifulSoup

from ..config import get_settings
from ..models import BookContentChunkCreate
from ..db.neon_client import get_neon_client
from ..db.qdrant_client import get_qdrant_client
from .embeddings import EmbeddingService

logger = logging.getLogger(__name__)
settings = get_settings()


class ContentIndexer:
    """Service for parsing, chunking, and indexing book content."""

    def __init__(self):
        self.tokenizer = tiktoken.get_encoding("cl100k_base")  # GPT-4 tokenizer
        self.embedding_service = EmbeddingService()

    def scan_markdown_files(self, content_path: str) -> List[Path]:
        """
        Scan directory for Markdown files.

        Args:
            content_path: Path to book content directory

        Returns:
            List of Path objects for .md files
        """
        content_dir = Path(content_path)
        if not content_dir.exists():
            raise FileNotFoundError(f"Content directory not found: {content_path}")

        # Find all .md files recursively
        md_files = sorted(content_dir.rglob("*.md"))
        logger.info(f"Found {len(md_files)} Markdown files in {content_path}")
        return md_files

    def parse_markdown_file(self, file_path: Path) -> Dict[str, Any]:
        """
        Parse a Markdown file and extract metadata and content.

        Args:
            file_path: Path to Markdown file

        Returns:
            Dict with chapter_name, section_name, content, and document_path
        """
        try:
            # Parse frontmatter and content
            with open(file_path, "r", encoding="utf-8") as f:
                post = frontmatter.load(f)

            # Extract metadata from frontmatter or headers
            chapter_name = post.get("title", None)
            section_name = post.get("section", None)
            page_number = post.get("page", None)

            # Get raw markdown content
            content = post.content

            # If no title in frontmatter, try to extract from first header
            if not chapter_name:
                # Find first # heading
                match = re.search(r"^#\s+(.+)$", content, re.MULTILINE)
                if match:
                    chapter_name = match.group(1).strip()
                else:
                    # Use filename as fallback
                    chapter_name = file_path.stem.replace("-", " ").title()

            # Convert relative path for document_path
            document_path = str(file_path)

            return {
                "chapter_name": chapter_name,
                "section_name": section_name,
                "page_number": page_number,
                "content": content,
                "document_path": document_path,
            }

        except Exception as e:
            logger.error(f"Failed to parse {file_path}: {e}")
            raise

    def chunk_text(
        self,
        text: str,
        chunk_size: int = None,
        overlap: int = None,
        min_chunk_size: int = None,
    ) -> List[str]:
        """
        Chunk text using recursive chunking with token-based splitting.
        Preserves code blocks intact per research.md recommendations.

        Args:
            text: Text to chunk
            chunk_size: Target chunk size in tokens (default from config)
            overlap: Overlap size in tokens (default from config)
            min_chunk_size: Minimum chunk size (default from config)

        Returns:
            List of text chunks
        """
        chunk_size = chunk_size or settings.chunk_size
        overlap = overlap or settings.chunk_overlap
        min_chunk_size = min_chunk_size or settings.min_chunk_size

        # Extract code blocks to preserve them
        code_blocks = []
        code_block_pattern = r"```[\s\S]*?```"

        def replace_code_block(match):
            placeholder = f"__CODE_BLOCK_{len(code_blocks)}__"
            code_blocks.append(match.group(0))
            return placeholder

        text_with_placeholders = re.sub(code_block_pattern, replace_code_block, text)

        # Split by paragraphs first (recursive approach)
        paragraphs = text_with_placeholders.split("\n\n")
        chunks = []
        current_chunk = []
        current_tokens = 0

        for paragraph in paragraphs:
            paragraph = paragraph.strip()
            if not paragraph:
                continue

            # Tokenize paragraph
            para_tokens = self.tokenizer.encode(paragraph)
            para_token_count = len(para_tokens)

            # If paragraph itself is larger than chunk_size, split it
            if para_token_count > chunk_size:
                # Save current chunk if exists
                if current_chunk:
                    chunks.append("\n\n".join(current_chunk))
                    current_chunk = []
                    current_tokens = 0

                # Split large paragraph by sentences
                sentences = re.split(r"(?<=[.!?])\s+", paragraph)
                for sentence in sentences:
                    sent_tokens = len(self.tokenizer.encode(sentence))
                    if current_tokens + sent_tokens > chunk_size and current_chunk:
                        chunks.append("\n\n".join(current_chunk))
                        # Keep overlap
                        overlap_text = current_chunk[-1] if current_chunk else ""
                        current_chunk = [overlap_text, sentence] if overlap_text else [sentence]
                        current_tokens = len(self.tokenizer.encode("\n\n".join(current_chunk)))
                    else:
                        current_chunk.append(sentence)
                        current_tokens += sent_tokens
            else:
                # Add paragraph to current chunk
                if current_tokens + para_token_count > chunk_size and current_chunk:
                    chunks.append("\n\n".join(current_chunk))
                    # Keep last paragraph for overlap
                    overlap_text = current_chunk[-1] if current_chunk else ""
                    current_chunk = [overlap_text, paragraph] if overlap_text else [paragraph]
                    current_tokens = len(self.tokenizer.encode("\n\n".join(current_chunk)))
                else:
                    current_chunk.append(paragraph)
                    current_tokens += para_token_count

        # Add remaining chunk
        if current_chunk:
            chunk_text = "\n\n".join(current_chunk)
            if len(self.tokenizer.encode(chunk_text)) >= min_chunk_size:
                chunks.append(chunk_text)

        # Restore code blocks
        restored_chunks = []
        for chunk in chunks:
            for i, code_block in enumerate(code_blocks):
                chunk = chunk.replace(f"__CODE_BLOCK_{i}__", code_block)
            restored_chunks.append(chunk)

        logger.info(f"Created {len(restored_chunks)} chunks from text")
        return restored_chunks

    async def index_file(self, file_path: Path) -> List[BookContentChunkCreate]:
        """
        Index a single Markdown file: parse, chunk, generate embeddings.

        Args:
            file_path: Path to Markdown file

        Returns:
            List of BookContentChunkCreate objects
        """
        # Parse file
        file_data = self.parse_markdown_file(file_path)

        # Chunk content
        chunks_text = self.chunk_text(file_data["content"])

        # Create chunk objects
        chunk_objects = []
        for idx, chunk_text in enumerate(chunks_text):
            # Count tokens
            token_count = len(self.tokenizer.encode(chunk_text))

            # Skip if too short
            if token_count < settings.min_chunk_size:
                continue

            # Generate embedding
            embedding_vector = await self.embedding_service.generate_embedding(chunk_text)

            # Create chunk object
            chunk = BookContentChunkCreate(
                text=chunk_text,
                embedding_vector=embedding_vector,
                chapter_name=file_data["chapter_name"],
                section_name=file_data["section_name"],
                page_number=file_data["page_number"],
                document_path=file_data["document_path"],
                chunk_index=idx,
                token_count=token_count,
                metadata={"file_name": file_path.name},
            )
            chunk_objects.append(chunk)

        logger.info(f"Indexed {len(chunk_objects)} chunks from {file_path.name}")
        return chunk_objects

    async def index_content(
        self, content_path: str, force_reindex: bool = False
    ) -> Dict[str, Any]:
        """
        Index all book content from a directory.

        Args:
            content_path: Path to book content directory
            force_reindex: If True, delete existing chunks before indexing

        Returns:
            Dict with indexing statistics
        """
        import time

        start_time = time.time()

        # Scan for files
        md_files = self.scan_markdown_files(content_path)

        if not md_files:
            logger.warning(f"No Markdown files found in {content_path}")
            return {
                "status": "failed",
                "chunks_indexed": 0,
                "failed_files": [],
                "processing_time_ms": 0,
            }

        # Force reindex: clear Qdrant collection
        if force_reindex:
            logger.info("Force reindex: clearing existing chunks")
            qdrant_client = get_qdrant_client()
            await qdrant_client.delete_collection()
            await qdrant_client.connect()  # Recreate collection

        # Index all files
        all_chunks = []
        failed_files = []

        for file_path in md_files:
            try:
                chunks = await self.index_file(file_path)
                all_chunks.extend(chunks)
            except Exception as e:
                logger.error(f"Failed to index {file_path}: {e}")
                failed_files.append(str(file_path))

        # Store in databases
        if all_chunks:
            await self.store_chunks(all_chunks)

        processing_time_ms = int((time.time() - start_time) * 1000)

        status = "success" if not failed_files else "partial_success" if all_chunks else "failed"

        return {
            "status": status,
            "chunks_indexed": len(all_chunks),
            "failed_files": failed_files,
            "processing_time_ms": processing_time_ms,
        }

    async def store_chunks(self, chunks: List[BookContentChunkCreate]):
        """
        Store chunks in Qdrant (vectors) and Neon Postgres (metadata).

        Args:
            chunks: List of BookContentChunkCreate objects
        """
        if not chunks:
            return

        qdrant_client = get_qdrant_client()
        neon_client = get_neon_client()

        # Prepare data for Qdrant
        chunk_ids = []
        vectors = []
        payloads = []

        for chunk in chunks:
            chunk_id = str(chunk.chunk_id) if hasattr(chunk, 'chunk_id') else str(uuid4())
            chunk_ids.append(chunk_id)
            vectors.append(chunk.embedding_vector)
            payloads.append({
                "chapter_name": chunk.chapter_name,
                "section_name": chunk.section_name,
                "text": chunk.text,
                "document_path": chunk.document_path,
            })

        # Store in Qdrant
        await qdrant_client.upsert_vectors(chunk_ids, vectors, payloads)

        # Store metadata in Neon Postgres
        async with neon_client.get_connection() as conn:
            for i, chunk in enumerate(chunks):
                await conn.execute(
                    """
                    INSERT INTO book_content_chunks
                    (chunk_id, text, chapter_name, section_name, page_number,
                     document_path, chunk_index, token_count, metadata)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
                    ON CONFLICT (document_path, chunk_index)
                    DO UPDATE SET
                        text = EXCLUDED.text,
                        chapter_name = EXCLUDED.chapter_name,
                        section_name = EXCLUDED.section_name,
                        page_number = EXCLUDED.page_number,
                        token_count = EXCLUDED.token_count,
                        metadata = EXCLUDED.metadata,
                        created_at = CURRENT_TIMESTAMP
                    """,
                    (
                        chunk_ids[i],
                        chunk.text,
                        chunk.chapter_name,
                        chunk.section_name,
                        chunk.page_number,
                        chunk.document_path,
                        chunk.chunk_index,
                        chunk.token_count,
                        chunk.metadata,
                    ),
                )

        logger.info(f"Stored {len(chunks)} chunks in databases")


# Convenience function
async def index_book_content(content_path: str, force_reindex: bool = False):
    """Index book content from a directory."""
    indexer = ContentIndexer()
    return await indexer.index_content(content_path, force_reindex)
