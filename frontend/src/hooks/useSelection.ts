/**
 * Hook for detecting text selection on the page.
 * Used for selected-text mode queries.
 */

import { useState, useEffect } from "react";

export interface SelectionState {
  text: string;
  hasSelection: boolean;
}

export const useSelection = (): SelectionState => {
  const [selection, setSelection] = useState<SelectionState>({
    text: "",
    hasSelection: false,
  });

  useEffect(() => {
    const handleSelectionChange = () => {
      const selectedText = window.getSelection()?.toString().trim() || "";

      setSelection({
        text: selectedText,
        hasSelection: selectedText.length >= 10, // Minimum 10 chars per validation
      });
    };

    // Listen for selection changes
    document.addEventListener("selectionchange", handleSelectionChange);

    return () => {
      document.removeEventListener("selectionchange", handleSelectionChange);
    };
  }, []);

  return selection;
};
