# Usage Guide

## Data Preparation
1. Place raw data in the `data/raw` folder.
2. Run the preprocessing script: `src/data/preprocess.py`
3. Run data augmentation (if needed):`src/data/augment.py`

## LLM Training
1. Adjust hyperparameters in `config/global_config.yaml`.
2. Run the training script:`bash scripts/run_llm_training.sh`

## Knowledge Distillation
1. Once training is complete, run:`bash scripts/run_distillation.sh`

## ROS Integration
1. Build the ROS package:`colcon build --packages-select slm_ros`
2. Source the workspace:`source install/setup.bash`
3. Deploy the ROS nodes:`bash scripts/deploy_ros.sh`
