import React from 'react';
import RagChatWidget from '../components/RagChatWidget';
import ReadingProgress from '../components/ReadingProgress';

// Root component that wraps the entire application
export default function Root({ children }) {
  return (
    <>
      <ReadingProgress />
      {children}
      <RagChatWidget />
    </>
  );
}