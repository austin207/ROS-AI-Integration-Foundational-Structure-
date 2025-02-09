#!/usr/bin/env python3
import os
import random

def augment_text(text):
    # Simple augmentation: randomly shuffle the words
    words = text.split()
    random.shuffle(words)
    return " ".join(words)

def augment_file(input_path, output_path, num_augments=5):
    with open(input_path, 'r', encoding='utf-8') as fin:
        text = fin.read()
    augmented_texts = [augment_text(text) for _ in range(num_augments)]
    with open(output_path, 'w', encoding='utf-8') as fout:
        for i, aug in enumerate(augmented_texts):
            fout.write(f"--- Augmentation {i+1} ---\n")
            fout.write(aug + "\n\n")
    print(f"Augmented data saved to {output_path}")

if __name__ == '__main__':
    input_file = os.path.join(os.path.dirname(__file__), "../../data/processed/sample_processed.txt")
    output_file = os.path.join(os.path.dirname(__file__), "../../data/augmentation/sample_augmented.txt")
    augment_file(input_file, output_file)
