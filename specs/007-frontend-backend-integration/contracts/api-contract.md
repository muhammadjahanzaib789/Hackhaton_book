# API Contract: Frontend-Backend Integration

**Version**: 1.0.0 | **Date**: 2025-12-24
**Purpose**: Define the required API endpoints and data structures for frontend-backend communication

## API Endpoints Contract

Every frontend-backend integration implementation MUST conform to this API contract.

### Authentication Endpoints

#### POST /api/auth/login
Authenticate user and return session token

**Request**:
```http
POST /api/auth/login
Content-Type: application/json

{
  "email": "string",                // User's email address
  "password": "string"              // User's password (hashed on client)
}
```

**Response** (Success - 200):
```json
{
  "user": {
    "id": "string",                 // User identifier
    "email": "string",              // User's email
    "name": "string",               // User's name
    "role": "string"                // User's role/permissions
  },
  "token": "string",                // JWT session token
  "expiresIn": "number",            // Token expiration in seconds
  "refreshToken": "string"          // Refresh token for session extension
}
```

**Response** (Error - 4xx):
```json
{
  "error": {
    "code": "string",               // Error code (e.g., "INVALID_CREDENTIALS")
    "message": "string",            // Human-readable error message
    "details": "object"             // Additional error details
  }
}
```

#### POST /api/auth/refresh
Refresh authentication token

**Request**:
```http
POST /api/auth/refresh
Content-Type: application/json
Authorization: Bearer {refresh_token}

{
  "refreshToken": "string"          // Refresh token from login
}
```

**Response** (Success - 200):
```json
{
  "token": "string",                // New JWT session token
  "expiresIn": "number"             // Token expiration in seconds
}
```

#### POST /api/auth/logout
End user session

**Request**:
```http
POST /api/auth/logout
Authorization: Bearer {access_token}
```

**Response** (Success - 200):
```json
{
  "message": "Successfully logged out"
}
```

### General API Endpoints

#### GET /api/{resource}
Retrieve a list of resources

**Request**:
```http
GET /api/{resource}?page={number}&limit={number}&sort={field}&order={asc|desc}
Authorization: Bearer {access_token}
```

**Response** (Success - 200):
```json
{
  "data": [
    {
      // Resource object
    }
  ],
  "pagination": {
    "page": "number",               // Current page
    "limit": "number",              // Items per page
    "total": "number",              // Total number of items
    "pages": "number"               // Total number of pages
  },
  "links": {
    "self": "string",               // Current page URL
    "first": "string",              // First page URL
    "prev": "string",               // Previous page URL (if exists)
    "next": "string",               // Next page URL (if exists)
    "last": "string"                // Last page URL
  }
}
```

#### GET /api/{resource}/{id}
Retrieve a specific resource by ID

**Response** (Success - 200):
```json
{
  "data": {
    // Resource object
  }
}
```

#### POST /api/{resource}
Create a new resource

**Request**:
```http
POST /api/{resource}
Content-Type: application/json
Authorization: Bearer {access_token}

{
  // Resource data to create
}
```

**Response** (Success - 201):
```json
{
  "data": {
    // Created resource object (with ID)
  },
  "message": "Resource created successfully"
}
```

#### PUT /api/{resource}/{id}
Update an existing resource

**Request**:
```http
PUT /api/{resource}/{id}
Content-Type: application/json
Authorization: Bearer {access_token}

{
  // Updated resource data
}
```

**Response** (Success - 200):
```json
{
  "data": {
    // Updated resource object
  },
  "message": "Resource updated successfully"
}
```

#### DELETE /api/{resource}/{id}
Delete a resource

**Response** (Success - 200):
```json
{
  "message": "Resource deleted successfully"
}
```

### Real-time Communication Endpoints

#### WebSocket Connection
Establish real-time communication channel

**Endpoint**: `ws://your-domain.com/api/realtime?token={jwt_token}`

**Message Format**:
```json
{
  "type": "string",                 // Message type (e.g., "subscribe", "unsubscribe", "data")
  "channel": "string",              // Channel to subscribe/unsubscribe from
  "payload": "object",              // Message payload
  "timestamp": "timestamp"          // When message was sent
}
```

**Server Response Format**:
```json
{
  "type": "string",                 // Message type (e.g., "ack", "error", "data")
  "requestId": "string",            // ID of the original request (if applicable)
  "payload": "object",              // Response payload
  "timestamp": "timestamp"          // When response was sent
}
```

### Error Response Contract

All API endpoints MUST return errors in this format:

```json
{
  "error": {
    "code": "string",               // Standardized error code
    "message": "string",            // Human-readable error message
    "details": "object",            // Additional error details
    "timestamp": "timestamp",       // When error occurred
    "requestId": "string",          // Unique identifier for the request
    "path": "string",               // API endpoint that generated the error
    "method": "string"              // HTTP method of the request
  }
}
```

### Standard HTTP Status Codes

- **200**: Success for GET, PUT, DELETE operations
- **201**: Success for POST operations (resource created)
- **204**: Success for operations with no content to return
- **400**: Bad request (invalid input, missing parameters)
- **401**: Unauthorized (authentication required or failed)
- **403**: Forbidden (authenticated but not authorized)
- **404**: Not found (resource does not exist)
- **409**: Conflict (resource already exists or constraint violation)
- **422**: Unprocessable entity (validation errors)
- **429**: Too many requests (rate limit exceeded)
- **500**: Internal server error
- **502**: Bad gateway (when proxying to other services)
- **503**: Service unavailable (temporary server unavailability)

## Request/Response Headers Contract

### Required Request Headers
- `Authorization: Bearer {token}` - For authenticated requests
- `Content-Type: application/json` - For JSON payloads
- `Accept: application/json` - To specify JSON response format

### Standard Response Headers
- `Content-Type: application/json` - For JSON responses
- `X-Request-ID: {uuid}` - Unique identifier for the request
- `X-RateLimit-Limit: {number}` - Rate limit ceiling for the endpoint
- `X-RateLimit-Remaining: {number}` - Remaining requests in current window
- `X-RateLimit-Reset: {timestamp}` - Time when rate limit resets

## Data Models

### Standard Response Format
```javascript
{
  // Success response
  "data": "object|array",           // The primary response data
  "message": "string",              // Optional: Human-readable message
  "meta": "object",                 // Optional: Additional metadata
  "links": "object"                 // Optional: Navigation links for paginated responses
}
```

### Standard Error Format
```javascript
{
  "error": {
    "code": "string",               // Standardized error code
    "message": "string",            // Human-readable error message
    "details": "object",            // Field-specific errors or additional details
    "timestamp": "timestamp",       // When error occurred
    "requestId": "string"           // Unique request identifier
  }
}
```

### Authentication Token Format
```javascript
{
  "header": {
    "alg": "string",                // Algorithm used (e.g., "HS256")
    "typ": "string"                 // Token type (e.g., "JWT")
  },
  "payload": {
    "sub": "string",                // Subject (user ID)
    "email": "string",              // User's email
    "name": "string",               // User's name
    "role": "string",               // User's role
    "iat": "timestamp",             // Issued at time
    "exp": "timestamp",             // Expiration time
    "jti": "string"                 // JWT ID for uniqueness
  }
}
```

## Validation Checklist

Before an API implementation is considered complete:

- [ ] All endpoints return appropriate HTTP status codes
- [ ] Request/response schemas match contract exactly
- [ ] Error responses follow the error contract
- [ ] All required headers are properly set
- [ ] Authentication is enforced on protected endpoints
- [ ] Rate limiting is implemented where appropriate
- [ ] All endpoints are documented with examples
- [ ] Pagination is implemented for list endpoints
- [ ] Quality Bar test: "Can a frontend developer integrate without guessing?"

## Implementation Requirements

### API Design Requirements
- MUST follow RESTful principles with proper HTTP methods
- MUST use consistent URL patterns and naming conventions
- MUST implement proper request validation and sanitization
- MUST include appropriate error handling and logging

### Security Requirements
- MUST enforce authentication on protected endpoints
- MUST validate JWT tokens properly
- MUST implement rate limiting to prevent abuse
- MUST sanitize all inputs to prevent injection attacks

### Performance Requirements
- MUST implement caching for frequently accessed data
- MUST compress responses for efficiency
- MUST handle concurrent requests appropriately
- MUST maintain response times under 500ms for 95% of requests

### Real-time Requirements
- MUST maintain WebSocket connections efficiently
- MUST implement proper reconnection logic
- MUST validate real-time messages before processing
- MUST handle connection failures gracefully