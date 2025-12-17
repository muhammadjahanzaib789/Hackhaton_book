/**
 * Loading Indicator Component
 * Shows spinner and estimated time during query processing.
 */

import React, { useState, useEffect } from "react";

interface LoadingIndicatorProps {
  message?: string;
}

export const LoadingIndicator: React.FC<LoadingIndicatorProps> = ({
  message = "Thinking...",
}) => {
  const [elapsed, setElapsed] = useState(0);

  useEffect(() => {
    const interval = setInterval(() => {
      setElapsed((prev) => prev + 1);
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  const estimatedTime = elapsed < 2 ? "2-3 seconds" : "a few more seconds";

  return (
    <div className="loading-indicator">
      <div className="loading-spinner">
        <div className="spinner"></div>
      </div>
      <div className="loading-text">
        <div className="loading-message">{message}</div>
        <div className="loading-estimate">
          {elapsed > 0 && `Estimated time: ${estimatedTime}`}
        </div>
      </div>
    </div>
  );
};
