import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatButton from '../components/Chatbot/FloatingChatButton';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingChatButton />
    </>
  );
}