# API Contract: RAG Chatbot Integration

**Version**: 1.0.0 | **Date**: 2025-12-24
**Purpose**: Define the required API endpoints and data structures for the RAG chatbot system

## API Endpoints Contract

Every RAG chatbot implementation MUST conform to this API contract.

### Document Management Endpoints

#### POST /api/documents/upload
Upload a document for RAG processing

**Request**:
```http
POST /api/documents/upload
Content-Type: multipart/form-data

FormData:
- file: [uploaded file]
- sessionId: [optional session identifier]
```

**Response** (Success - 200):
```json
{
  "documentId": "string",
  "fileName": "string",
  "fileType": "string",
  "fileSize": "number",
  "status": "processing|processed|failed",
  "message": "string"
}
```

**Response** (Error - 4xx/5xx):
```json
{
  "error": "string",
  "message": "string"
}
```

#### GET /api/documents
List documents in the current session

**Request**:
```http
GET /api/documents?sessionId={sessionId}
```

**Response** (Success - 200):
```json
{
  "documents": [
    {
      "documentId": "string",
      "fileName": "string",
      "fileType": "string",
      "uploadDate": "timestamp",
      "status": "processing|processed|failed",
      "pageCount": "number"
    }
  ]
}
```

#### DELETE /api/documents/{documentId}
Remove a document from the knowledge base

**Request**:
```http
DELETE /api/documents/{documentId}
```

**Response** (Success - 200):
```json
{
  "message": "Document deleted successfully"
}
```

### Chat and RAG Endpoints

#### POST /api/chat/start
Initialize a new chat session

**Request**:
```http
POST /api/chat/start
Content-Type: application/json

{
  "sessionId": "optional string"
}
```

**Response** (Success - 200):
```json
{
  "sessionId": "string",
  "timestamp": "timestamp",
  "message": "Session initialized"
}
```

#### POST /api/chat/message
Send a message and receive a RAG-enhanced response

**Request**:
```http
POST /api/chat/message
Content-Type: application/json

{
  "sessionId": "string",
  "message": "string",
  "documentIds": ["string"]  // optional, restrict to specific documents
}
```

**Response** (Success - 200):
```json
{
  "responseId": "string",
  "message": "string",
  "timestamp": "timestamp",
  "citations": [
    {
      "documentId": "string",
      "documentName": "string",
      "text": "string",
      "page": "number",
      "confidence": "number"  // 0-1
    }
  ],
  "retrievedContext": [
    {
      "documentId": "string",
      "content": "string",
      "score": "number"  // relevance score
    }
  ]
}
```

#### GET /api/chat/history/{sessionId}
Retrieve chat history for a session

**Request**:
```http
GET /api/chat/history/{sessionId}
```

**Response** (Success - 200):
```json
{
  "sessionId": "string",
  "history": [
    {
      "id": "string",
      "type": "user|assistant",
      "message": "string",
      "timestamp": "timestamp",
      "citations": ["citation object if assistant"]
    }
  ]
}
```

### System Status Endpoints

#### GET /api/health
Check system health

**Response** (Success - 200):
```json
{
  "status": "healthy",
  "timestamp": "timestamp",
  "services": {
    "database": "connected",
    "vectorStore": "connected",
    "llm": "available"
  }
}
```

## Data Models

### Document Model
```javascript
{
  documentId: "string (UUID)",      // Unique identifier
  fileName: "string",               // Original file name
  fileType: "string",               // MIME type or extension
  fileSize: "number",               // Size in bytes
  uploadDate: "timestamp",          // When document was uploaded
  status: "enum",                   // 'processing', 'processed', 'failed'
  pageCount: "number",              // Number of pages (if applicable)
  metadata: "object",               // Additional format-specific metadata
  sessionId: "string (UUID)"        // Associated session
}
```

### Chat Message Model
```javascript
{
  messageId: "string (UUID)",       // Unique identifier
  sessionId: "string (UUID)",       // Associated session
  type: "enum",                     // 'user', 'assistant'
  content: "string",                // Message content
  timestamp: "timestamp",           // When message was created
  citations: [Citation],            // References to source documents
  retrievedContext: [ContextItem]    // Context used for generation
}
```

### Citation Model
```javascript
{
  documentId: "string (UUID)",      // Source document
  documentName: "string",           // Source document name
  text: "string",                   // Quoted text from source
  page: "number",                   // Page number (if applicable)
  confidence: "number",             // Confidence score (0-1)
  position: "number"                // Position in source (if applicable)
}
```

### Context Item Model
```javascript
{
  documentId: "string (UUID)",      // Source document
  content: "string",                // Retrieved text content
  score: "number",                  // Relevance score (0-1)
  metadata: "object"                // Additional retrieval metadata
}
```

## Error Response Contract

All API endpoints MUST return errors in this format:

```json
{
  "error": "string",                // Error code or type
  "message": "string",              // Human-readable error message
  "details": "object",              // Optional additional error details
  "timestamp": "timestamp",         // When error occurred
  "requestId": "string"             // ID of the failed request
}
```

## Validation Checklist

Before an API implementation is considered complete:

- [ ] All endpoints return appropriate HTTP status codes
- [ ] Request/response schemas match contract exactly
- [ ] Error responses follow the error contract
- [ ] All required fields are present in responses
- [ ] Optional fields are properly handled
- [ ] API rate limiting is implemented where appropriate
- [ ] Authentication/authorization is implemented per security requirements
- [ ] All endpoints are documented with examples
- [ ] Quality Bar test: "Can a frontend developer integrate without guessing?"

## Implementation Requirements

### Document Processing Requirements
- MUST support file sizes up to 10MB
- MUST process PDF, DOCX, TXT, HTML formats
- MUST extract text content while preserving structure where possible
- MUST store document metadata for retrieval optimization

### RAG Requirements
- MUST retrieve relevant passages within 3 seconds for documents under 100 pages
- MUST include citations in all document-related responses
- MUST maintain conversation context across multiple turns
- MUST return confidence scores for retrieved information

### Session Management Requirements
- MUST isolate document knowledge bases between sessions
- MUST preserve conversation history within sessions
- MUST handle concurrent user sessions appropriately
- MUST clean up old sessions to prevent resource exhaustion