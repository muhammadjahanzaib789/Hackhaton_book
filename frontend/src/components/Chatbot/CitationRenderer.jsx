import React from 'react';

const CitationRenderer = ({ citations }) => {
  if (!citations || citations.length === 0) {
    return null;
  }

  return (
    <div className="citations-container">
      <h4>References:</h4>
      <ul className="citations-list">
        {citations.map((citation, index) => (
          <li key={citation.id || index} className="citation-item">
            <div className="citation-header">
              <a
                href={citation.url}
                target="_blank"
                rel="noopener noreferrer"
                className="citation-link"
              >
                {citation.document_title || citation.source_document}
                {citation.page_number && `, Page ${citation.page_number}`}
              </a>
              {citation.relevance_score && (
                <span className="relevance-score" title={`Relevance: ${(citation.relevance_score * 100).toFixed(1)}%`}>
                  {(citation.relevance_score * 100).toFixed(1)}%
                </span>
              )}
            </div>
            {citation.chunk_text && (
              <div className="citation-text">
                "{citation.chunk_text.substring(0, 200) + (citation.chunk_text.length > 200 ? '...' : '')}"
              </div>
            )}
            {citation.section_title && (
              <div className="section-title">
                Section: {citation.section_title}
              </div>
            )}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default CitationRenderer;