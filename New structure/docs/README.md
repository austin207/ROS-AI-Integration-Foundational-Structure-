# Domain-Specific LLM and ROS Integration Project

This project builds a large language model (LLM) entirely from scratch on domain-specific data (electronics, programming, VLSI) and distills it into a small language model (SLM) for integration within a ROS system.

## Components
- **LLM Training:** Custom Transformer model trained from scratch.
- **Knowledge Distillation:** Compressing the LLM into an efficient SLM using teacher–student methods.
- **ROS Integration:** Deploying the SLM as a ROS node to enable natural language interaction for robotic tasks.

## Folder Structure
- `data/` – Raw and processed data plus augmentation scripts.
- `models/` – Model checkpoints (trained and distilled).
- `src/` – Source code for data processing, model training, distillation, ROS integration, and evaluation.
- `scripts/` – Shell scripts for automation.
- `config/` – Global configuration files.
- `docs/` – Documentation.
- `tests/` – Unit and integration tests.

Please refer to the [Usage Guide](usage.md) for detailed instructions.

## License
This project is licensed under the BSD License.
