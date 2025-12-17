# Data Model: Frontend and Backend Integration for Embedded RAG Chatbot

## Entities

### Chat Query
**Description**: A user's question submitted to the RAG system
- **query_text**: String (required) - The text of the user's question
- **mode**: String (required) - Query mode ("full_book" or "selected_text")
- **selected_text**: String (optional) - Text selected by user when in selected_text mode
- **metadata**: Object (optional) - Additional metadata for tracking

**Validation rules**:
- query_text must be non-empty
- mode must be one of the allowed values
- selected_text must be provided when mode is "selected_text"

### Chat Response
**Description**: The AI-generated answer from the backend
- **answer**: String (required) - The response text from the AI
- **citations**: Array of Citation objects - References to source material
- **query**: String (required) - The original query that generated this response
- **tokens_used**: Integer (optional) - Number of tokens in the response
- **confidence**: Float (optional) - Confidence score of the response

**Validation rules**:
- answer must be non-empty
- citations must be an array of valid Citation objects

### Citation
**Description**: A reference to source material in the book
- **id**: String (required) - Unique identifier for the citation
- **source_document**: String (required) - Name of the source document
- **document_title**: String (optional) - Title of the document
- **page_number**: Integer (optional) - Page number where content appears
- **section_title**: String (optional) - Title of the section
- **chunk_text**: String (required) - The actual text content of the citation
- **relevance_score**: Float (required) - Relevance score between 0.0 and 1.0
- **url**: String (required) - URL to navigate to the source
- **position_in_document**: Integer (optional) - Position within the document

**Validation rules**:
- All required fields must be present
- relevance_score must be between 0.0 and 1.0
- url must be a valid URL format

### Chat Session
**Description**: Temporary context for the conversation
- **session_id**: String (optional) - Unique identifier for the session
- **messages**: Array of Message objects - History of messages in the session
- **current_mode**: String (required) - Current query mode
- **selected_text**: String (optional) - Currently selected text context

**Validation rules**:
- current_mode must be one of the allowed values
- messages must be an array of valid Message objects

### Message
**Description**: A single message in the chat session
- **id**: String (required) - Unique identifier for the message
- **role**: String (required) - Role of the message ("user" or "assistant")
- **content**: String (required) - Content of the message
- **timestamp**: DateTime (required) - When the message was created
- **citations**: Array of Citation objects (optional) - Citations associated with the message

**Validation rules**:
- role must be one of the allowed values
- content must be non-empty
- timestamp must be a valid date/time

## Relationships

- Chat Session contains multiple Messages
- Chat Query may include Citations
- Chat Response contains multiple Citations
- Message may include Citations