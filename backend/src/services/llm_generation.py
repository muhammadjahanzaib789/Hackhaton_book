"""
LLM generation service using OpenRouter API.
Generates answers to user queries based on retrieved book content.
Includes mode-specific guardrails for selected-text constraints.
"""

import logging
import asyncio
from typing import List, Optional
from openai import OpenAI

from ..config import get_settings
from ..models.session import SessionMode
from .retrieval import RetrievalResult

logger = logging.getLogger(__name__)
settings = get_settings()

# Timeout for LLM requests (5 seconds per US4 requirements)
LLM_TIMEOUT_SECONDS = 5


class LLMGenerationService:
    """Service for generating answers using LLM via OpenRouter."""

    def __init__(self):
        # Initialize OpenAI client with OpenRouter base URL
        self.client = OpenAI(
            base_url=settings.openrouter_base_url,
            api_key=settings.openrouter_api_key,
        )
        self.model = settings.llm_model  # e.g., openai/gpt-4-turbo-preview

    async def generate_answer(
        self,
        query: str,
        retrieved_chunks: List[RetrievalResult],
        mode: SessionMode,
        selected_text: Optional[str] = None,
    ) -> str:
        """
        Generate an answer to the user's query based on retrieved content.

        Args:
            query: User's question
            retrieved_chunks: List of relevant book chunks
            mode: Session mode (full_book or selected_text)
            selected_text: Original selected text (required for selected_text mode)

        Returns:
            Generated answer as string

        Raises:
            Exception: If LLM generation fails
        """
        try:
            # Build context from retrieved chunks
            context = self._build_context(retrieved_chunks)

            # Build prompt based on mode
            if mode == SessionMode.SELECTED_TEXT:
                system_prompt = self._build_selected_text_system_prompt()
                user_prompt = self._build_selected_text_user_prompt(
                    query, selected_text, context
                )
            else:  # SessionMode.FULL_BOOK
                system_prompt = self._build_full_book_system_prompt()
                user_prompt = self._build_full_book_user_prompt(query, context)

            logger.info(f"Generating answer for query: {query[:100]}...")

            # Call OpenRouter LLM with timeout enforcement
            try:
                # Wrap synchronous call in async timeout
                response = await asyncio.wait_for(
                    asyncio.to_thread(
                        self.client.chat.completions.create,
                        model=self.model,
                        messages=[
                            {"role": "system", "content": system_prompt},
                            {"role": "user", "content": user_prompt},
                        ],
                        temperature=0.7,
                        max_tokens=500,
                        timeout=LLM_TIMEOUT_SECONDS,
                    ),
                    timeout=LLM_TIMEOUT_SECONDS,
                )

                # Extract answer
                answer = response.choices[0].message.content.strip()

                logger.info(f"Generated answer length: {len(answer)} characters")
                return answer

            except asyncio.TimeoutError:
                logger.error(f"LLM request timed out after {LLM_TIMEOUT_SECONDS}s")
                raise TimeoutError(
                    f"Answer generation took too long (>{LLM_TIMEOUT_SECONDS}s). Please try again with a simpler question."
                )

        except TimeoutError:
            # Re-raise timeout errors
            raise

        except Exception as e:
            logger.error(f"LLM generation failed: {e}", exc_info=True)
            raise

    def _build_context(self, chunks: List[RetrievalResult]) -> str:
        """
        Build context string from retrieved chunks.

        Args:
            chunks: List of RetrievalResult objects

        Returns:
            Formatted context string
        """
        if not chunks:
            return "No relevant content found."

        context_parts = []
        for i, chunk in enumerate(chunks, 1):
            # Format: [Chunk 1 - Chapter Name, Section Name]
            header = f"[Chunk {i}"
            if chunk.chapter_name:
                header += f" - {chunk.chapter_name}"
            if chunk.section_name:
                header += f", {chunk.section_name}"
            if chunk.page_number:
                header += f", Page {chunk.page_number}"
            header += f"] (Relevance: {chunk.relevance_score:.2f})"

            context_parts.append(f"{header}\n{chunk.text}\n")

        return "\n".join(context_parts)

    def _build_full_book_system_prompt(self) -> str:
        """Build system prompt for full-book mode."""
        return """You are a knowledgeable assistant helping readers understand content from the book "Physical AI & Humanoid Robotics".

Your task is to answer questions based on the retrieved book content provided in the context. Follow these guidelines:

1. **Answer based on the provided context**: Use the retrieved chunks to formulate your answer.
2. **Be accurate and specific**: Reference specific concepts, chapters, or sections from the book when relevant.
3. **Be concise**: Aim for clear, focused answers (2-3 paragraphs maximum).
4. **Acknowledge limitations**: If the context doesn't contain enough information to fully answer the question, say so clearly.
5. **Use natural language**: Write in a helpful, educational tone suitable for readers learning about Physical AI and robotics.

Do not make up information or use knowledge outside the provided context unless the context is insufficient."""

    def _build_full_book_user_prompt(self, query: str, context: str) -> str:
        """Build user prompt for full-book mode."""
        return f"""**Retrieved Book Content:**
{context}

**User's Question:**
{query}

**Instructions:**
Answer the user's question based on the retrieved book content above. Be clear, accurate, and concise."""

    def _build_selected_text_system_prompt(self) -> str:
        """Build system prompt for selected-text mode with strict guardrails."""
        return """You are a focused assistant helping readers understand a specific passage they've selected from the book "Physical AI & Humanoid Robotics".

**CRITICAL CONSTRAINT**: You must ONLY answer using the provided selected text. Do NOT use prior knowledge or information from other parts of the book.

Your task is to explain or clarify the selected text in response to the user's question. Follow these guidelines:

1. **Strictly stay within the selected text**: Only reference information that appears in the selected passage.
2. **Clarify and explain**: Help the user understand the selected text by breaking down concepts, rephrasing, or highlighting key points.
3. **Be concise**: Keep your answer focused and brief (1-2 paragraphs).
4. **Acknowledge if the question can't be answered**: If the selected text doesn't contain information needed to answer the question, explicitly say: "The selected text doesn't contain enough information to answer this question."

Do NOT introduce external knowledge or information from other book chapters."""

    def _build_selected_text_user_prompt(
        self, query: str, selected_text: str, context: str
    ) -> str:
        """Build user prompt for selected-text mode."""
        return f"""**Selected Text (from the book):**
{selected_text}

**Retrieved Related Content (for additional context only):**
{context}

**User's Question:**
{query}

**Instructions:**
Answer the user's question based PRIMARILY on the selected text above. You may use the retrieved related content for additional context, but your answer must focus on and stay within the bounds of what's in the selected text. If the selected text doesn't contain the information needed to answer the question, clearly state this limitation."""


# Global instance
llm_service = LLMGenerationService()


def get_llm_service() -> LLMGenerationService:
    """Get the global LLM generation service instance."""
    return llm_service
