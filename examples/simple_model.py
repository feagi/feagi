#!/usr/bin/env python
"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
A simple example demonstrating how to use FEAGI to create and train a model.

This example creates a simple model and demonstrates basic operations like
training, prediction, and evaluation.
"""

from feagi import FEAGI


def main():
    """Run the example."""
    print("Creating a FEAGI instance...")
    feagi = FEAGI()

    print("\nCreating a simple model...")
    model = feagi.create_model("example_model")
    print(f"Model created: {model.name} (type: {model.model_type})")

    print("\nAvailable models:")
    for model_name in feagi.list_models():
        print(f"- {model_name}")

    # Create some dummy training data
    print("\nPreparing training data...")
    training_data = [
        {"features": [1.0, 2.0, 3.0], "label": 0},
        {"features": [4.0, 5.0, 6.0], "label": 1},
        {"features": [7.0, 8.0, 9.0], "label": 2},
    ]

    print("\nTraining the model...")
    metrics = model.train(training_data, epochs=5)
    print(f"Training completed with metrics: {metrics}")

    # Create some dummy test data
    print("\nPreparing test data...")
    test_data = [
        {"features": [1.1, 2.1, 3.1]},
        {"features": [4.1, 5.1, 6.1]},
    ]

    print("\nMaking predictions...")
    predictions = model.predict(test_data)
    for i, pred in enumerate(predictions):
        print(f"Prediction {i + 1}: {pred:.4f}")

    print("\nEvaluating the model...")
    eval_metrics = model.evaluate(training_data)
    print(f"Evaluation metrics: {eval_metrics}")

    print("\nSaving the model...")
    model_path = model.save("example_model.feagi")
    print(f"Model saved to: {model_path}")

    print("\nExample completed successfully!")


if __name__ == "__main__":
    main()
