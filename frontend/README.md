# RAG Chatbot Frontend

React + TypeScript chat widget for the integrated RAG chatbot, embedded in the Physical AI & Humanoid Robotics book.

## Features

- **Two Query Modes**:
  - Full-Book: Search entire indexed content
  - Selected-Text: Ask about highlighted passages only
- **Real-time Text Selection**: Auto-detects when user highlights text
- **Source Citations**: Clickable citations that navigate to book sections
- **Hover Previews**: Shows full excerpt on citation hover
- **Responsive Design**: Works on desktop and mobile
- **Dark/Light Mode**: Matches Docusaurus theme
- **Character Counter**: 5-500 character validation
- **Error Handling**: User-friendly error messages

## Tech Stack

- React 18
- TypeScript 5.x
- Axios for API calls
- CSS3 with CSS variables for theming

## Installation

```bash
npm install
```

## Development

```bash
npm start
```

Runs the widget in development mode at http://localhost:3001

## Build

```bash
npm run build
```

Creates production build in `dist/` directory.

## Integration with Docusaurus

The widget is designed to be embedded in the Docusaurus book site via theme swizzling.

### Steps:

1. **Build the widget**:
```bash
npm run build
```

2. **Create Docusaurus theme wrapper**:

In `physical-ai-book/src/theme/Root.tsx`:
```tsx
import React from "react";
import { ChatWidget } from "../../../frontend/src/components/ChatWidget";
import "../../../frontend/src/styles/chat-widget.css";

export default function Root({ children }) {
  const apiUrl = process.env.REACT_APP_API_URL || "http://localhost:8000/v1";

  return (
    <>
      {children}
      <ChatWidget apiUrl={apiUrl} />
    </>
  );
}
```

3. **Configure environment**:

In `physical-ai-book/.env`:
```env
REACT_APP_API_URL=http://localhost:8000/v1
```

4. **Start Docusaurus**:
```bash
cd physical-ai-book
npm start
```

## Project Structure

```
frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget.tsx          # Main container component
│   │   ├── ChatMessage.tsx         # User/assistant message display
│   │   ├── ChatInput.tsx           # Input field with mode selector
│   │   ├── SourceCitation.tsx      # Citation card with navigation
│   │   └── LoadingIndicator.tsx    # Loading spinner with ETA
│   ├── hooks/
│   │   ├── useChat.ts              # Chat state management
│   │   └── useSelection.ts         # Text selection detection
│   ├── services/
│   │   └── api.ts                  # Axios API client
│   ├── types/
│   │   └── chat.ts                 # TypeScript interfaces
│   ├── styles/
│   │   └── chat-widget.css         # Complete widget styling
│   └── index.ts                    # Public exports
├── package.json
├── tsconfig.json
└── README.md
```

## Components

### ChatWidget
Main container with floating button and chat interface.

**Props**:
- `apiUrl?: string` - Backend API URL (default: env variable)

**Features**:
- Minimize/maximize
- Clear chat history
- Auto-scroll to latest message
- Error banner
- Empty state

### ChatMessage
Displays individual user/assistant messages with sources.

**Props**:
- `message: ChatMessageType` - Message object with content, sources, timestamp

**Features**:
- Different styling for user vs assistant
- Mode badge (full-book / selected-text)
- Source citations list
- Timestamp

### ChatInput
Textarea with mode selector and send button.

**Props**:
- `onSendMessage: (query, mode, selectedText?) => void`
- `isLoading: boolean`
- `selectedText: string`
- `hasSelection: boolean`

**Features**:
- Character counter (5-500)
- Mode radio buttons
- Auto-switch to selected-text mode when text is highlighted
- Selection indicator badge
- Disabled state during loading
- Enter to send (Shift+Enter for newline)

### SourceCitation
Citation card with chapter/section info and navigation.

**Props**:
- `citation: SourceCitationType` - Citation data
- `index: number` - Citation number

**Features**:
- Relevance confidence indicator (high/medium/low)
- Clickable link to book section
- Hover tooltip with full excerpt
- Arrow icon for navigation

### LoadingIndicator
Spinner with estimated time message.

**Props**:
- `message?: string` - Loading message (default: "Thinking...")

**Features**:
- Animated spinner
- Elapsed time counter
- Estimated completion time

## Hooks

### useChat
Manages chat state and API communication.

**Returns**:
```typescript
{
  messages: ChatMessage[],
  isLoading: boolean,
  error: string | null,
  sessionId: string | null,
  sendMessage: (query, mode, selectedText?) => Promise<void>,
  clearMessages: () => void,
  clearError: () => void
}
```

**Features**:
- Message history management
- Session persistence
- Input validation (5-500 chars, selected text ≥10 chars)
- Error handling with user-friendly messages
- Async API calls

### useSelection
Detects text selection on the page.

**Returns**:
```typescript
{
  text: string,
  hasSelection: boolean
}
```

**Features**:
- Listens to `selectionchange` events
- Minimum 10 character threshold
- Auto-cleanup on unmount

## API Client

### apiClient (`src/services/api.ts`)

**Methods**:

```typescript
// Submit query
await apiClient.query({
  session_id: "uuid",
  query: "What is ROS 2?",
  mode: SessionMode.FULL_BOOK,
  selected_text: null
});

// Health check
await apiClient.health();

// Index content (admin)
await apiClient.indexContent("/path/to/docs", false);
```

**Error Handling**:
- 400: Invalid request
- 404: No relevant content
- 429: Rate limit exceeded
- 500: Server error / timeout
- 503: Service unavailable
- Network errors

## Styling

The widget uses CSS variables for theming, matching Docusaurus:

```css
:root {
  --chat-primary: #3b82f6;
  --chat-bg: #ffffff;
  --chat-text: #1f2937;
  ...
}

[data-theme="dark"] {
  --chat-bg: #1e293b;
  --chat-text: #f1f5f9;
  ...
}
```

**Responsive Design**:
- Desktop: 420px × 600px floating widget
- Mobile: Full-screen overlay

## Usage Examples

### Basic Query (Full-Book Mode)
```typescript
import { ChatWidget } from "./components/ChatWidget";

function App() {
  return <ChatWidget apiUrl="http://localhost:8000/v1" />;
}
```

### Programmatic Query Submission
```typescript
import { useChat } from "./hooks/useChat";
import { SessionMode } from "./types/chat";

function CustomChat() {
  const { sendMessage, messages, isLoading } = useChat();

  const handleAsk = () => {
    sendMessage("What is ROS 2?", SessionMode.FULL_BOOK);
  };

  return (
    <div>
      <button onClick={handleAsk} disabled={isLoading}>
        Ask Question
      </button>
      {messages.map(msg => (
        <div key={msg.id}>{msg.content}</div>
      ))}
    </div>
  );
}
```

### Custom Styling
```css
/* Override default colors */
.chat-widget-container {
  --chat-primary: #8b5cf6; /* Purple theme */
}

/* Adjust widget position */
.chat-widget-button,
.chat-widget-container {
  bottom: 100px; /* Move higher */
}
```

## Testing

### Manual Testing
1. Start backend: `cd backend && uvicorn src.main:app --reload`
2. Start frontend: `npm start`
3. Test scenarios:
   - Full-book query
   - Selected-text query
   - Citation navigation
   - Error handling (disconnect backend)

### E2E Testing
```bash
npm run test:e2e
```

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `REACT_APP_API_URL` | Backend API URL | `http://localhost:8000/v1` |

## Browser Support

- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## Accessibility

- ARIA labels on buttons
- Keyboard navigation support
- Screen reader friendly
- Focus management

## Performance

- Lazy loading of components
- Debounced selection detection
- Optimized re-renders with React.memo
- Minimal bundle size (~50KB gzipped)

## Troubleshooting

### Widget not appearing
- Check console for errors
- Verify CSS file is imported
- Ensure Root.tsx is in correct location

### "Unable to connect to server"
- Verify backend is running
- Check CORS configuration in backend
- Verify API URL is correct

### Selection not detected
- Check browser supports `selectionchange` event
- Verify minimum 10 characters selected
- Try refreshing the page

## Contributing

1. Follow existing code style
2. Add TypeScript types for all props
3. Test in both light and dark modes
4. Ensure mobile responsiveness
5. Update README for new features

## License

MIT

## Support

- Backend README: `../backend/README.md`
- Integration Guide: `../INTEGRATION_GUIDE.md`
- Specification: `../specs/001-integrated-rag-chatbot/spec.md`
