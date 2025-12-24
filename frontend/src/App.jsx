import React from 'react';
import Chatbot from './components/Chatbot/Chatbot';
import './App.css';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <h1>Welcome to My Website</h1>
        <p>This is an example website with an integrated chatbot.</p>
        <p>Click the chat icon in the bottom right corner to open the chatbot.</p>
      </header>
      
      {/* Main website content */}
      <main className="main-content">
        <section className="content-section">
          <h2>About Our Services</h2>
          <p>We provide excellent customer service and support. Our AI assistant is available 24/7 to help answer your questions.</p>
        </section>
        
        <section className="content-section">
          <h2>Features</h2>
          <ul>
            <li>Real-time customer support</li>
            <li>Instant responses to common questions</li>
            <li>Available 24/7</li>
            <li>Easy to use interface</li>
          </ul>
        </section>
      </main>
      
      {/* Chatbot Component - Will appear as a floating icon that expands when clicked */}
      <Chatbot />
    </div>
  );
}

export default App;