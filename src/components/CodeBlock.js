import React, { useState } from 'react';

const CodeBlock = ({ children, className = '', ...props }) => {
  const [copied, setCopied] = useState(false);
  const codeString = React.Children.toArray(children).map(child => child.props.children || child).join('');

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(codeString);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy text: ', err);
    }
  };

  return (
    <div className="code-block-wrapper">
      <pre className={className} {...props}>
        <code>{children}</code>
      </pre>
      <button
        className={`copy-button ${copied ? 'copied' : ''}`}
        onClick={handleCopy}
        aria-label="Copy code"
      >
        {copied ? '✓ Copied!' : 'Copy'}
      </button>
    </div>
  );
};

export default CodeBlock;