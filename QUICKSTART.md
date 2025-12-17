# Quick Start Guide - See the Chatbot in Your Book

This guide will help you quickly start the chatbot and see it running in your Physical AI & Humanoid Robotics book.

## Prerequisites

Make sure you have:
- Python 3.11+ installed
- Node.js 18+ installed
- Git bash or terminal

## Step 1: Set Up Backend Environment

1. Create backend `.env` file:

```bash
cd C:/Users/Sheheryar/OneDrive/Desktop/loop/backend
```

Create `.env` with these settings:
```env
# OpenRouter API (you need to get your API key from https://openrouter.ai/)
OPENROUTER_API_KEY=your_api_key_here
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
EMBEDDING_MODEL=qwen/qwen3-embedding-8b
LLM_MODEL=openai/gpt-4-turbo-preview

# Qdrant Cloud (you need to create a free account at https://cloud.qdrant.io/)
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here

# Neon Postgres (you need to create a free account at https://neon.tech/)
DATABASE_URL=your_neon_connection_string_here

# API Configuration
ENVIRONMENT=development
CORS_ORIGINS=http://localhost:3000,http://localhost:8000
CHUNK_SIZE=400
CHUNK_OVERLAP=80
MIN_CHUNK_SIZE=100
```

2. Install Python dependencies:

```bash
pip install -r requirements.txt
```

## Step 2: Start the Backend

```bash
cd C:/Users/Sheheryar/OneDrive/Desktop/loop/backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

You should see:
```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Database connections initialized successfully
INFO:     Background task scheduler started
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

**Test backend**: Open http://localhost:8000/v1/health

## Step 3: Index Book Content (One-Time)

In a new terminal:

```bash
curl -X POST http://localhost:8000/v1/admin/index \
  -H "Content-Type: application/json" \
  -d "{\"content_path\": \"C:/Users/Sheheryar/OneDrive/Desktop/loop/docs\", \"force_reindex\": false}"
```

This will take a few minutes to index all book content.

Expected output:
```json
{
  "status": "success",
  "chunks_indexed": 1024,
  "failed_files": [],
  "processing_time_ms": 45000
}
```

## Step 4: Set Up Frontend Environment

```bash
cd C:/Users/Sheheryar/OneDrive/Desktop/loop/physical-ai-book
```

Create `.env` file:
```env
REACT_APP_API_URL=http://localhost:8000/v1
```

## Step 5: Install and Start Docusaurus

```bash
cd C:/Users/Sheheryar/OneDrive/Desktop/loop/physical-ai-book
npm install
npm start
```

The book will open at http://localhost:3000

## Step 6: See the Chatbot!

1. **Look for the floating chat button** in the bottom-right corner (blue circle with chat icon)

2. **Click the button** to open the chat widget

3. **Try a full-book query**:
   - Type: "What are the main components of a humanoid robot?"
   - Click Send
   - See the answer with source citations!

4. **Try selected-text mode**:
   - Go to any page in the book
   - Highlight any paragraph
   - The chat button will show a "!" badge
   - Open the chat
   - "Ask About Selection" mode is auto-selected
   - Type: "Explain this in simpler terms"
   - The answer will ONLY use your selected text!

5. **Click on source citations** to navigate to the referenced book sections

## Troubleshooting

### Backend won't start
- Check that port 8000 is not already in use
- Verify your `.env` file has valid API keys
- Check logs for specific error messages

### "Unable to connect to server" in chat
- Make sure backend is running (http://localhost:8000/v1/health)
- Check CORS settings in backend `.env`
- Verify REACT_APP_API_URL in frontend `.env`

### Chat button doesn't appear
- Clear browser cache and reload
- Check browser console for errors (F12)
- Verify Root.tsx exists in physical-ai-book/src/theme/

### No relevant content found
- Make sure you ran the indexing step (Step 3)
- Check backend logs to confirm chunks were indexed
- Try different query phrasing

### Rate limit exceeded
- Wait 1 minute (20 requests per minute limit)
- Or restart the backend to reset counters

## What You'll See

**Chat Widget Features**:
- ✅ Floating button in bottom-right
- ✅ Minimize/maximize/close buttons
- ✅ Mode selector (Full Book / Ask About Selection)
- ✅ Character counter (5-500 chars)
- ✅ Loading spinner with estimated time
- ✅ Source citations with:
  - Chapter and section names
  - Relevance confidence (%)
  - Text preview
  - Clickable navigation links
  - Hover for full excerpt
- ✅ Dark/light mode matching Docusaurus theme

**Try These Queries**:

Full-Book Mode:
- "What is ROS 2 and why is it important?"
- "Explain inverse kinematics"
- "What are the challenges in humanoid robot control?"

Selected-Text Mode:
- Select any technical paragraph
- Ask: "What does this mean?"
- Ask: "Give me an example"
- Ask: "Why is this important?"

## Next Steps

Once you've tested locally:
1. Review the analytics: Backend logs show query patterns
2. Check session cleanup: Sessions auto-cleanup after 30 min
3. Test rate limiting: Try 21 queries in 1 minute
4. Deploy to production (see deployment.md)

## Getting API Keys

### OpenRouter
1. Go to https://openrouter.ai/
2. Sign up for free account
3. Get API key from dashboard
4. Free tier: 50 requests/day
5. Add $10 credit for 1000 requests/day

### Qdrant Cloud
1. Go to https://cloud.qdrant.io/
2. Sign up for free account
3. Create a cluster (free tier: 1GB)
4. Copy cluster URL and API key

### Neon Postgres
1. Go to https://neon.tech/
2. Sign up for free account
3. Create a project (free tier: 0.5GB)
4. Copy connection string

## Support

If you encounter issues:
1. Check logs: Backend terminal shows detailed error messages
2. Review documentation: backend/README.md, frontend/README.md
3. Test health endpoint: http://localhost:8000/v1/health
4. Check API docs: http://localhost:8000/docs

Enjoy your AI-powered book! 🎉
