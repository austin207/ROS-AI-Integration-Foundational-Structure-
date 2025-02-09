#!/usr/bin/env python3
import os
import re

def clean_text(text):
    # Convert to lowercase and remove extra whitespace
    text = text.lower()
    text = re.sub(r'\s+', ' ', text)
    return text.strip()

def preprocess_file(input_path, output_path):
    with open(input_path, 'r', encoding='utf-8') as fin:
        text = fin.read()
    cleaned_text = clean_text(text)
    with open(output_path, 'w', encoding='utf-8') as fout:
        fout.write(cleaned_text)
    print(f"Preprocessed file saved to {output_path}")

if __name__ == '__main__':
    input_file = os.path.join(os.path.dirname(__file__), "../../data/raw/sample.txt")
    output_file = os.path.join(os.path.dirname(__file__), "../../data/processed/sample_processed.txt")
    preprocess_file(input_file, output_file)
