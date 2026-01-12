import React, { useState, useEffect } from 'react';

const ReadingProgress = () => {
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    const updateProgress = () => {
      const scrollTop = window.scrollY;
      const scrollHeight = document.documentElement.scrollHeight - window.innerHeight;
      const progressPercentage = scrollHeight > 0 ? (scrollTop / scrollHeight) * 100 : 0;
      setProgress(progressPercentage);
    };

    window.addEventListener('scroll', updateProgress);

    // Initial calculation
    updateProgress();

    return () => window.removeEventListener('scroll', updateProgress);
  }, []);

  // Only show on documentation pages
  const isDocPage = typeof window !== 'undefined' && window.location.pathname.startsWith('/docs/');

  if (!isDocPage) {
    return null;
  }

  return (
    <div className="reading-progress-container">
      <div
        className="reading-progress-bar"
        style={{ width: `${progress}%` }}
      />
    </div>
  );
};

export default ReadingProgress;