#!/usr/bin/env python3
import torch
import math
from llm_training.model import TransformerModel
from llm_training.utils import get_data_iterator

def evaluate_model(model, data_iter, vocab_size, criterion):
    model.eval()
    total_loss = 0.
    count = 0
    with torch.no_grad():
        for src, targets in data_iter:
            output = model(src)
            loss = criterion(output.view(-1, vocab_size), targets.view(-1))
            total_loss += loss.item()
            count += 1
            if count >= 10:  # Evaluate on 10 batches as an example
                break
    return math.exp(total_loss / count)

if __name__ == '__main__':
    vocab_size = 10000
    model = TransformerModel(vocab_size)
    criterion = torch.nn.CrossEntropyLoss()
    data_iter = get_data_iterator("../../data/processed/train.txt", 32, vocab_size)
    perplexity = evaluate_model(model, data_iter, vocab_size, criterion)
    print(f"Model Perplexity: {perplexity:.2f}")
