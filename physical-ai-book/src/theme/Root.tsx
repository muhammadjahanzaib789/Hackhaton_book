/**
 * Docusaurus Root Theme Wrapper
 * Swizzled Root component to inject the ChatWidget on all pages.
 */

import React from "react";
import ChatbotInterface from "@site/src/components/Chatbot/ChatbotInterface";

// Import the chat widget CSS
import "@site/src/css/chatbot.css";

export default function Root({ children }: { children: React.ReactNode }) {
  // For Docusaurus, we can pass the API URL through component props if needed
  // For now, the configuration is handled in the component itself
  return (
    <>
      {children}
      {/* Add the chatbot interface to all pages */}
      <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 1000 }}>
        <ChatbotInterface />
      </div>
    </>
  );
}
