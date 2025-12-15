#!/usr/bin/env python3
"""
Code Example: [Example Title]

Problem Solved: [What problem does this code solve? Why would someone need it?]

Assumptions:
- [Assumption 1: e.g., ROS 2 Humble is installed]
- [Assumption 2: e.g., Required packages available]
- [Assumption 3: e.g., Running on Ubuntu 22.04]

Failure Modes:
- [Failure 1]: [Symptom] → [Cause] → [Fix]
- [Failure 2]: [Symptom] → [Cause] → [Fix]
- [Failure 3]: [Symptom] → [Cause] → [Fix]

Input: [What input does this code expect? Format, range, etc.]
Output: [What output does this produce? Format, where it goes, etc.]

Usage:
    [How to run this code]
    [Example command]
    [Expected result]

Dependencies:
    pip install [packages]
    # or
    apt install [packages]

Related Examples:
    - [example_name.py]: [How it relates]
    - [other_example.py]: [How it relates]
"""

# Standard library imports first
import sys
import time

# Third-party imports next (alphabetical)
# import numpy as np
# import rclpy

# Local imports last
# from my_package import my_module


class ExampleClass:
    """
    Brief description of what this class does.

    Why this class? [Explain the design decision]

    Attributes:
        attribute_name: Description of what it stores and why
    """

    def __init__(self, parameter: str):
        """
        Initialize the example.

        Args:
            parameter: Description of what this parameter controls
        """
        # Why store this? [Brief explanation]
        self.parameter = parameter

    def do_something(self) -> bool:
        """
        Brief description of method purpose.

        Why this method? [Explain when/why to call it]

        Returns:
            True if successful, False otherwise

        Raises:
            ValueError: If parameter is invalid
        """
        # Implementation with comments explaining WHY not just WHAT
        # Why this check? [Explanation]
        if not self.parameter:
            raise ValueError("Parameter cannot be empty")

        # Why this approach? [Explanation]
        result = self._process_internally()

        return result

    def _process_internally(self) -> bool:
        """Private helper method - explain why it's separate."""
        return True


def main():
    """
    Main entry point demonstrating usage.

    This function shows the typical usage pattern for this code example.
    """
    print("Starting example...")

    try:
        # Create instance with explanation of parameter choice
        example = ExampleClass(parameter="demo")

        # Perform action with explanation of why
        success = example.do_something()

        if success:
            print("Example completed successfully")
        else:
            print("Example completed with warnings")

    except ValueError as e:
        print(f"Configuration error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        sys.exit(2)


if __name__ == '__main__':
    main()
