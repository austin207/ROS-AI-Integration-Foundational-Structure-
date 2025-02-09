#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

# Dummy SLM model class for inference; replace with your actual SLM implementation.
class SLMSimpleModel:
    def __init__(self, checkpoint_path):
        # Simulate model loading.
        self.checkpoint_path = checkpoint_path
        print(f"Loaded SLM model from {checkpoint_path}")

    def infer(self, input_text):
        # Dummy inference: reverse the input text.
        return input_text[::-1]

class SLMNode(Node):
    def __init__(self):
        super().__init__('slm_node')
        self.subscription = self.create_subscription(
            String,
            'slm_input',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'slm_output', 10)
        # Load the distilled SLM model from checkpoint.
        checkpoint = os.path.join(os.path.dirname(__file__), '../../../models/distilled/checkpoint_epoch_1.pt')
        self.model = SLMSimpleModel(checkpoint)
        self.get_logger().info("SLM Node initialized and model loaded.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received input: {msg.data}")
        result = self.model.infer(msg.data)
        out_msg = String()
        out_msg.data = result
        self.publisher.publish(out_msg)
        self.get_logger().info(f"Published output: {out_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
