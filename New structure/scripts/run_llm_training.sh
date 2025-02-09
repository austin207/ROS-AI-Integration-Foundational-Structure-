#!/bin/bash
# Run LLM training from scratch
echo "Starting LLM training..."
python3 src/llm_training/train.py --config config/global_config.yaml
