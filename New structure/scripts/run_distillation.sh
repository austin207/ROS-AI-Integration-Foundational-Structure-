#!/bin/bash
# Run knowledge distillation
echo "Starting knowledge distillation..."
python3 src/distillation/distill.py --config config/global_config.yaml
