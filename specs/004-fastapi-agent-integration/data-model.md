# Data Model: Chatbot UI in Docusaurus Integration

## Entity Models

### 1. ChatMessage (Pydantic BaseModel)
- **Purpose**: Structure for individual chat messages exchanged between UI and backend
- **Fields**:
  - `id: str` - Unique identifier for the message
  - `role: str` - Role of the message sender ("user" or "assistant")
  - `content: str` - The actual content of the message
  - `timestamp: datetime` - When the message was created
  - `sources: List[str]` - List of sources referenced in the response (for retrieval context)

### 2. ChatRequest (Pydantic BaseModel)
- **Purpose**: Structure for chat requests sent from frontend to backend
- **Fields**:
  - `query: str` - The user's query/question
  - `conversation_id: Optional[str]` - ID to maintain conversation context (for future expansion)
  - `context: Optional[Dict]` - Additional context for the query (for future expansion)

### 3. ChatResponse (Pydantic BaseModel)
- **Purpose**: Structure for responses from the backend to the frontend
- **Fields**:
  - `response: str` - The agent's response to the query
  - `sources: List[Dict]` - Sources used to generate the response (retrieval context)
  - `query: str` - Echo of the original query
  - `timestamp: datetime` - When the response was generated
  - `conversation_id: str` - ID to maintain conversation context
  - `retrieval_results: List[Dict]` - Detailed retrieval results used to generate the response

### 4. ChatHistory (Pydantic BaseModel)
- **Purpose**: Structure for maintaining conversation history (for future expansion)
- **Fields**:
  - `conversation_id: str` - Unique identifier for the conversation
  - `messages: List[ChatMessage]` - List of messages in chronological order
  - `created_at: datetime` - When the conversation was started
  - `updated_at: datetime` - When the conversation was last updated

## API Contract Models

### 1. ChatEndpointRequest
- **Purpose**: Request model for the /chat endpoint
- **Structure**:
  - `POST /chat`
  - `Content-Type: application/json`
  - `Body`: ChatRequest model

### 2. ChatEndpointResponse
- **Purpose**: Response model for the /chat endpoint
- **Structure**:
  - `Response-Type: application/json`
  - `Status Codes`: 200 (success), 400 (bad request), 500 (server error)
  - `Body`: ChatResponse model

## Relationships

### 1. ChatRequest connects to:
- ChatResponse (one-to-one relationship for each query-response pair)

### 2. ChatHistory contains:
- Multiple ChatMessage entities (one-to-many relationship)

### 3. ChatResponse includes:
- Multiple source references from the retrieval system (one-to-many relationship)

## Validation Rules

### 1. ChatRequest Validation:
- `query` field must not be empty
- `query` length must be between 1 and 2000 characters
- `conversation_id` must be a valid UUID if provided

### 2. ChatMessage Validation:
- `role` must be either "user" or "assistant"
- `content` must not be empty
- `timestamp` must be in ISO format

### 3. ChatResponse Validation:
- `response` must not be empty
- `query` must match the original query
- `sources` must be a valid list of source objects

## State Transitions

### 1. Message States:
- `CREATED` → When a message is first generated
- `PROCESSING` → While the agent is generating a response
- `COMPLETED` → When the response is ready
- `ERROR` → If an error occurs during processing

## Data Flow

### 1. Query Processing Flow:
```
Frontend sends ChatRequest → FastAPI receives → agent.py processes query → retrieval.py provides context → agent.py generates response → FastAPI returns ChatResponse → Frontend displays response
```

### 2. History Management Flow:
```
New query → Add to client-side history → Send to backend → Receive response → Add response to history → Update UI
```