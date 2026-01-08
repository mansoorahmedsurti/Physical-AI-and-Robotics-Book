# RAG Chat Widget

A React chat widget for the RAG (Retrieval-Augmented Generation) chatbot backend.

## Installation

```bash
npm install path/to/rag_backend/react_widget
```

## Usage

```jsx
import React from 'react';
import { RagChatWidget } from 'path/to/rag_backend/react_widget';

function App() {
  return (
    <div className="App">
      <main>
        {/* Your existing content */}
      </main>

      {/* RAG Chat Widget */}
      <RagChatWidget
        backendUrl="http://localhost:8000"
        documentContext="book-content"
      />
    </div>
  );
}

export default App;
```

## Props

- `backendUrl` (string, optional): URL of the RAG backend API. Default: `'http://localhost:8000'`
- `documentContext` (string, optional): Context identifier for the documents. Default: `'general'`

## Features

- Floating chat widget that can be toggled open/closed
- Real-time conversation with the RAG backend
- Shows sources for AI-generated responses
- Loading indicators during API calls
- Ability to start new conversations
- Responsive design that works on mobile and desktop

## API Integration

The widget communicates with the RAG backend using the following endpoints:

- `POST /api/v1/chat/` - Send messages and receive responses
- `POST /api/v1/chat/conversation/` - Start new conversations