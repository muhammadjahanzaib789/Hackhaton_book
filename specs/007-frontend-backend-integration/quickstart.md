# Quickstart: Frontend and Backend Integration for Embedded RAG Chatbot

## Prerequisites

- Node.js 18+ for Docusaurus frontend
- Python 3.11+ for FastAPI backend
- Access to the RAG backend API (FastAPI service)
- Docusaurus 3.x project already set up

## Backend Setup

1. Ensure the RAG agent endpoints are available:
   - `/v1/agent/query` - For submitting queries to the RAG agent
   - `/v1/agent/health` - For checking agent service health

2. Verify the backend is running and accessible:
   ```bash
   curl http://localhost:8000/v1/agent/health
   ```

## Frontend Integration

1. Install required dependencies in your Docusaurus project:
   ```bash
   npm install axios # for API calls
   ```

2. Create the chatbot components in `src/components/Chatbot/`:
   - `ChatbotInterface.jsx` - Main chat interface component
   - `ChatMessage.jsx` - Individual message display
   - `ChatInput.jsx` - Input field with mode selector
   - `CitationRenderer.jsx` - Citation display component

3. Add the chatbot to your Docusaurus pages by importing and using the component:
   ```jsx
   import ChatbotInterface from '@site/src/components/Chatbot/ChatbotInterface';

   // In your page component
   <ChatbotInterface />
   ```

## Configuration

1. Set the backend API URL in your frontend configuration:
   ```javascript
   // In src/config/api.js or similar
   export const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/v1';
   ```

2. Configure environment variables for different environments:
   ```bash
   # .env.development
   REACT_APP_API_URL=http://localhost:8000/v1

   # .env.production
   REACT_APP_API_URL=https://your-api-domain.com/v1
   ```

## API Usage

### Submitting a Query

```javascript
// Example API call to submit a query
const submitQuery = async (queryText, mode = 'full_book', selectedText = null) => {
  const response = await axios.post('/agent/query', {
    query: queryText,
    mode: mode,
    selected_text: selectedText
  });

  return response.data;
};
```

### Query Modes

- **full_book**: Search across the entire book content
- **selected_text**: Limit responses to the provided selected text

## Testing the Integration

1. Start your Docusaurus development server:
   ```bash
   npm start
   ```

2. Start your FastAPI backend:
   ```bash
   python -m uvicorn src.main:app --reload
   ```

3. Navigate to a book page with the embedded chatbot
4. Try submitting a query in both full_book and selected_text modes
5. Verify that responses include proper citations with working links

## Troubleshooting

### API Connection Issues
- Verify the backend is running and accessible
- Check that the API URL is correctly configured
- Ensure CORS is properly configured on the backend

### Selected Text Not Working
- Verify that text selection detection is working properly
- Check that the selected text is being passed to the query correctly

### Citation Links Not Working
- Verify that citation URLs match the Docusaurus routing structure
- Check that document paths are correctly formatted