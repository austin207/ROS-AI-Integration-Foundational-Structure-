#!/usr/bin/env python3
import unittest
import torch
from src.llm_training.model import TransformerModel

class TestTransformerModel(unittest.TestCase):
    def test_forward_pass(self):
        vocab_size = 10000
        model = TransformerModel(vocab_size)
        sample_input = torch.randint(0, vocab_size, (35, 16))
        output = model(sample_input)
        self.assertEqual(output.shape, (35, 16, vocab_size))

if __name__ == '__main__':
    unittest.main()
