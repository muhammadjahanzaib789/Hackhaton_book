import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

// This is a layout wrapper that adds the chatbot to all pages
export default function LayoutWrapper(props) {
  return (
    <>
      {props.children}
      <Chatbot />
    </>
  );
}