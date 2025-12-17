# Research: Frontend and Backend Integration for Embedded RAG Chatbot

## Decision: Frontend Technology Stack
**Rationale**: Using React components within Docusaurus for the chatbot interface since Docusaurus is built on React and allows custom components. This provides seamless integration with the existing book pages.

**Alternatives considered**:
- Vanilla JavaScript components: Would require more custom code and wouldn't integrate as well with React-based Docusaurus
- Web components: Would work but add complexity without significant benefits over React components
- iFrame embedding: Would create isolation but complicate communication with parent page

## Decision: API Communication Pattern
**Rationale**: Using REST API calls from frontend to backend with JSON payload. This follows the constraint of HTTP/JSON over REST and works well with the existing FastAPI backend.

**Alternatives considered**:
- GraphQL: Would provide more flexible queries but requires additional setup and doesn't add significant value for this use case
- WebSockets: Would enable real-time communication but adds complexity for a question-response pattern
- Server-Sent Events: Could work for response streaming but REST is simpler for this use case

## Decision: Selected Text Handling
**Rationale**: Using JavaScript to detect and capture selected text on the current page, then sending it as part of the query payload when in selected-text mode. This allows for context-specific queries without complex state management.

**Alternatives considered**:
- Highlighting mechanism: Would require additional UI elements and state tracking
- Manual text input: Would be less user-friendly than native selection
- Page context only: Would be less flexible than allowing specific text selection

## Decision: Citation Link Format
**Rationale**: Using the existing citation format from the backend with Docusaurus-compatible URLs. This maintains consistency with existing citation functionality while ensuring links work within the book structure.

**Alternatives considered**:
- Custom citation format: Would require changes to backend response format
- Pop-up references: Would change the user experience from direct navigation
- Modal displays: Would show content in-place rather than navigating to source

## Decision: State Management
**Rationale**: Using React hooks for local component state management for the chat session. This keeps the implementation simple and leverages React's built-in capabilities.

**Alternatives considered**:
- Redux: Would add complexity for simple state needs
- Context API: Would be overkill for local component state
- Local storage: Would persist across sessions but might not be needed for temporary chat state

## Decision: Error Handling Strategy
**Rationale**: Implementing graceful error handling with user-friendly messages when backend API is unavailable. This addresses the edge case of API failures while maintaining good UX.

**Alternatives considered**:
- Silent failure: Would not inform users of issues
- Page refresh: Would be disruptive to user experience
- Modal error dialogs: Would interrupt user flow more than inline messages