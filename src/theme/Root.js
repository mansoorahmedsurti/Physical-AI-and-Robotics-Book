import React from 'react';
import RagChatWidget from '../components/RagChatWidget';

// Root component that wraps the entire application
export default function Root({ children }) {
  return (
    <>
      {children}
      <RagChatWidget />
    </>
  );
}