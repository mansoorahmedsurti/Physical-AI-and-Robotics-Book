import React, { useState, useEffect } from 'react';

const TypewriterEffect = () => {
  const phrases = [
    '> Initializing ROS 2...',
    '> Loading Isaac Sim...',
    '> Connecting VLA Models...',
    '> Training RL Policies...',
    '> Physical AI & Humanoid Robotics: The Future of Embodied Intelligence'
  ];

  const [currentPhraseIndex, setCurrentPhraseIndex] = useState(0);
  const [currentText, setCurrentText] = useState('');
  const [isDeleting, setIsDeleting] = useState(false);
  const [typingSpeed, setTypingSpeed] = useState(100);

  useEffect(() => {
    const currentPhrase = phrases[currentPhraseIndex];

    if (isDeleting) {
      // Deleting phase
      if (currentText.length > 0) {
        const timeout = setTimeout(() => {
          setCurrentText(currentText.slice(0, -1));
        }, 50);
        return () => clearTimeout(timeout);
      } else {
        // Finished deleting, move to next phrase
        setIsDeleting(false);
        setTypingSpeed(100);
        setCurrentPhraseIndex((prev) => (prev + 1) % phrases.length);
      }
    } else {
      // Typing phase
      if (currentText.length < currentPhrase.length) {
        const timeout = setTimeout(() => {
          setCurrentText(currentPhrase.substring(0, currentText.length + 1));
        }, typingSpeed);
        return () => clearTimeout(timeout);
      } else {
        // Finished typing, pause then start deleting
        const pauseTimeout = setTimeout(() => {
          if (currentPhraseIndex < phrases.length - 1) {
            setIsDeleting(true);
          } else {
            // For the final phrase, just keep the cursor blinking
            setTypingSpeed(300);
          }
        }, 2000);
        return () => clearTimeout(pauseTimeout);
      }
    }
  }, [currentText, isDeleting, currentPhraseIndex, phrases]);

  return (
    <div className="typewriter-container">
      <span className="typewriter-text">{currentText}</span>
      <span className="typewriter-cursor">|</span>
    </div>
  );
};

export default TypewriterEffect;