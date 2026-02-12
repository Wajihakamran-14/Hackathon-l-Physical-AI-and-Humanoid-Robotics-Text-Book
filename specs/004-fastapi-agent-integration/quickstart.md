# Quickstart Guide: Chatbot UI in Docusaurus Integration

## Prerequisites

- Python 3.11+
- Node.js 18+
- Access to Qdrant instance with existing collection
- Cohere API key for text embeddings
- OpenRouter API key for LLM responses

## Setup

### 1. Backend Setup

1. Navigate to the project directory:
   ```bash
   cd D:\Q-4\Hackathon-Physical-AI-and-Humanoid-Robotics
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up environment variables in `.env`:
   ```bash
   # Create .env file with connection details
   QDRANT_URL=your-qdrant-url
   QDRANT_API_KEY=your-api-key
   QDRANT_COLLECTION=your-collection-name
   COHERE_API_KEY=your-cohere-api-key
   OPENROUTER_API_KEY=your-openrouter-api-key
   ```

4. Start the FastAPI backend:
   ```bash
   python backend/start_api.py
   ```

   The API will be available at `http://localhost:8000`

### 2. Frontend Setup

1. Navigate to the book directory:
   ```bash
   cd book
   ```

2. Install Node.js dependencies:
   ```bash
   npm install
   ```

3. Update the Docusaurus configuration to include the chatbot component:
   - Modify `docusaurus.config.ts` to add the chatbot component
   - Add necessary imports and configuration for the chatbot UI

4. Start the Docusaurus development server:
   ```bash
   npm start
   ```

   The documentation site will be available at `http://localhost:3000`

## API Usage

### Chat Endpoint

Send a query to the chatbot:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?"
  }'
```

Response format:
```json
{
  "response": "ROS 2 (Robot Operating System 2) is the next-generation...",
  "sources": [...],
  "query": "What is ROS 2?",
  "timestamp": "2025-12-30T00:00:00Z",
  "conversation_id": "123e4567-e89b-12d3-a456-426614174000",
  "retrieval_results": [...]
}
```

### Health Check

Verify the API is running:
```bash
curl http://localhost:8000/health
```

## Integration Points

### 1. Docusaurus Chatbot Component

The chatbot UI is implemented as a React component that can be embedded in Docusaurus pages:

- Located in `book/src/components/Chatbot/`
- Communicates with the backend via the `/chat` endpoint
- Displays conversation history and retrieval context

### 2. Agent Integration

The backend reuses existing agent logic:

- `backend/agent.py` - RAG agent logic
- `backend/retrieval.py` - Qdrant retrieval logic
- No code duplication - all existing functionality is reused

## Testing

### Backend Tests

Run backend tests to verify API functionality:
```bash
pytest backend/ -v
```

### Frontend Tests

Run frontend tests for the chatbot UI:
```bash
cd book
npm test
```

## Troubleshooting

### Common Issues

1. **API Connection Errors**
   - Verify Qdrant connection details in `.env`
   - Check that Qdrant instance is accessible
   - Ensure API keys are correct

2. **CORS Issues**
   - Make sure the FastAPI server allows requests from the Docusaurus frontend
   - Check CORS configuration in `backend/api.py`

3. **Slow Response Times**
   - Verify network connectivity to Qdrant and Cohere APIs
   - Check that the OpenRouter API key is properly configured

### Verification Steps

1. Confirm the backend is running:
   ```bash
   curl http://localhost:8000/health
   ```

2. Test a simple query:
   ```bash
   curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "Hello"}'
   ```

3. Verify the Docusaurus site is accessible at `http://localhost:3000`