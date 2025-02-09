#!/usr/bin/env python3
import unittest
from src.data.preprocess import clean_text

class TestPreprocess(unittest.TestCase):
    def test_clean_text(self):
        sample = "  Hello   World!  "
        cleaned = clean_text(sample)
        self.assertEqual(cleaned, "hello world!")

if __name__ == '__main__':
    unittest.main()
