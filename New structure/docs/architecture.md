# Project Architecture

This project has three main components:

1. **LLM Training from Scratch:**  
   A custom Transformer-based LLM is built and trained using domain-specific data.

2. **Knowledge Distillation:**  
   The trained LLM serves as a teacher to distill its knowledge into a smaller, resource-efficient SLM.

3. **ROS Integration:**  
   The distilled SLM is deployed as a ROS node, enabling natural language command processing for robotic tasks.

Each component is modular and well separated to ensure scalability and ease of development.
