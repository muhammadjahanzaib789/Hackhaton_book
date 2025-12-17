#!/usr/bin/env python3
"""
Test script for RAG Retrieval Validation feature.

This script tests the retrieval functionality by making API calls to the backend.
"""
import asyncio
import httpx

async def test_retrieval():
    """Test the retrieval functionality."""
    print("Testing RAG Retrieval Validation feature...")

    async with httpx.AsyncClient(base_url="http://localhost:8000", timeout=30.0) as client:
        try:
            # Test query
            query_data = {
                "query_text": "What is gradient descent?",
                "top_k": 5,
                "similarity_threshold": 0.5
            }

            print(f"Sending query: {query_data}")

            response = await client.post("/v1/retrieval/query", json=query_data)

            print(f"Response status: {response.status_code}")
            if response.status_code == 200:
                print(f"Response: {response.json()}")
                print("✓ Retrieval test passed!")
            else:
                print(f"Response: {response.text}")
                print(f"✗ Retrieval test failed with status {response.status_code}")

        except Exception as e:
            print(f"✗ Error during retrieval test: {str(e)}")

async def test_validation():
    """Test the validation functionality."""
    print("\nTesting RAG Validation functionality...")

    async with httpx.AsyncClient(base_url="http://localhost:8000", timeout=30.0) as client:
        try:
            # Test validation benchmark
            validation_data = {
                "benchmark_name": "test_benchmark",
                "question_set": ["What is gradient descent?", "How does backpropagation work?"],
                "ground_truth_mappings": {
                    "What is gradient descent?": ["chunk_1", "chunk_2"],
                    "How does backpropagation work?": ["chunk_3", "chunk_4"]
                },
                "metrics": ["precision@5", "recall@5", "MRR"]
            }

            print(f"Sending validation request: {validation_data}")

            response = await client.post("/v1/validation/run", json=validation_data)

            print(f"Response status: {response.status_code}")
            if response.status_code == 200:
                print(f"Response: {response.json()}")
                print("✓ Validation test passed!")
            else:
                print(f"Response: {response.text}")
                print(f"✗ Validation test failed with status {response.status_code}")

        except Exception as e:
            print(f"✗ Error during validation test: {str(e)}")

async def main():
    """Main test function."""
    print("Starting RAG Retrieval and Validation tests...")

    # Test retrieval
    await test_retrieval()

    # Test validation
    await test_validation()

    print("\nTests completed!")

if __name__ == "__main__":
    asyncio.run(main())