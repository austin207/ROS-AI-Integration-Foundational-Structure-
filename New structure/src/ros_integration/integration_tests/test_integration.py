#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String

class TestSLMNode(Node):
    def __init__(self):
        super().__init__('test_slm_node')
        self.received = None
        self.subscription = self.create_subscription(
            String,
            'slm_output',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(String, 'slm_input', 10)

    def callback(self, msg):
        self.received = msg.data
        self.get_logger().info(f"Test received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    test_node = TestSLMNode()
    test_input = String()
    test_input.data = "hello world"
    test_node.publisher.publish(test_input)
    time.sleep(2)  # wait for propagation
    if test_node.received:
        print("Integration test passed, received:", test_node.received)
    else:
        print("Integration test failed.")
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
