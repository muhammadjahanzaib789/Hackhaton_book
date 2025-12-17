/**
 * Source Citation Component
 * Displays a citation with chapter/section info and clickable navigation link.
 * Shows excerpt on hover for preview.
 */

import React, { useState } from "react";
import { SourceCitation as SourceCitationType } from "../types/chat";

interface SourceCitationProps {
  citation: SourceCitationType;
  index: number;
}

export const SourceCitation: React.FC<SourceCitationProps> = ({ citation, index }) => {
  const [showTooltip, setShowTooltip] = useState(false);

  const handleClick = () => {
    if (citation.link) {
      // Navigate to the book location
      window.location.href = citation.link;
    }
  };

  const confidenceLevel =
    citation.relevance_score >= 0.8
      ? "high"
      : citation.relevance_score >= 0.5
      ? "medium"
      : "low";

  return (
    <div
      className={`source-citation ${citation.link ? "clickable" : ""}`}
      onClick={handleClick}
      onMouseEnter={() => setShowTooltip(true)}
      onMouseLeave={() => setShowTooltip(false)}
    >
      <div className="citation-header">
        <span className="citation-number">[{index + 1}]</span>
        <span className="citation-title">{citation.chapter_name}</span>
        {citation.section_name && (
          <span className="citation-section"> - {citation.section_name}</span>
        )}
      </div>

      <div className="citation-meta">
        {citation.page_number && (
          <span className="citation-page">Page {citation.page_number}</span>
        )}
        <span className={`citation-confidence ${confidenceLevel}`}>
          {(citation.relevance_score * 100).toFixed(0)}% relevant
        </span>
      </div>

      <div className="citation-preview">{citation.text_preview}</div>

      {showTooltip && citation.excerpt && (
        <div className="citation-tooltip">
          <div className="tooltip-content">{citation.excerpt}</div>
        </div>
      )}

      {citation.link && (
        <div className="citation-link-icon">
          <svg
            width="16"
            height="16"
            viewBox="0 0 16 16"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M8 1L14 8L8 15M14 8H2"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
          </svg>
        </div>
      )}
    </div>
  );
};
