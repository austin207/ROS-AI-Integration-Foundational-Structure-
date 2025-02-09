#!/usr/bin/env python3
import torch
import torch.nn as nn
import torch.optim as optim
from model import TransformerModel
from utils import get_data_iterator, save_checkpoint
import yaml
import os

def train_model(config):
    vocab_size = config['model']['vocab_size']
    model = TransformerModel(
        vocab_size=vocab_size,
        d_model=config['model']['d_model'],
        nhead=config['model']['nhead'],
        num_layers=config['model']['num_layers'],
        dim_feedforward=config['model']['dim_feedforward'],
        dropout=config['model']['dropout']
    )

    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=config['training']['lr'])
    num_epochs = config['training']['epochs']

    # Dummy data iterator; replace with actual data loader.
    data_iter = get_data_iterator(config['data']['train_file'], config['training']['batch_size'], vocab_size)

    model.train()
    for epoch in range(1, num_epochs + 1):
        total_loss = 0.
        for batch, (src, targets) in enumerate(data_iter):
            optimizer.zero_grad()
            output = model(src)
            loss = criterion(output.view(-1, vocab_size), targets.view(-1))
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
            if batch % config['training']['log_interval'] == 0 and batch > 0:
                print(f"Epoch {epoch}, Batch {batch}, Loss: {loss.item():.4f}")
            # For demonstration, break after a fixed number of batches.
            if batch >= 100:
                break
        avg_loss = total_loss / (batch + 1)
        print(f"Epoch {epoch} completed. Average Loss: {avg_loss:.4f}")
        save_checkpoint(model, optimizer, epoch, config['training']['checkpoint_dir'])
    return model

if __name__ == '__main__':
    with open('config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    if not os.path.exists(config['training']['checkpoint_dir']):
        os.makedirs(config['training']['checkpoint_dir'])
    trained_model = train_model(config)
