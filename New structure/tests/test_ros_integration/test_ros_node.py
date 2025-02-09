#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class DummySLMNodeTest(Node):
    def __init__(self):
        super().__init__('dummy_test_node')
        self.received = None
        self.subscription = self.create_subscription(String, 'slm_output', self.callback, 10)

    def callback(self, msg):
        self.received = msg.data

class TestROSIntegration(unittest.TestCase):
    def test_slm_node(self):
        rclpy.init()
        test_node = DummySLMNodeTest()
        publisher = test_node.create_publisher(String, 'slm_input', 10)
        test_msg = String()
        test_msg.data = "test command"
        publisher.publish(test_msg)
        time.sleep(2)  # wait for propagation
        self.assertIsNotNone(test_node.received)
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
