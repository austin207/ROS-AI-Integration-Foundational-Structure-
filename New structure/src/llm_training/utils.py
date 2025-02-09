#!/usr/bin/env python3
import torch
import os

def get_data_iterator(file_path, batch_size, vocab_size):
    # Dummy iterator: yields random tensors.
    seq_len = 35
    while True:
        src = torch.randint(0, vocab_size, (seq_len, batch_size))
        targets = torch.randint(0, vocab_size, (seq_len, batch_size))
        yield src, targets

def save_checkpoint(model, optimizer, epoch, checkpoint_dir):
    checkpoint_path = os.path.join(checkpoint_dir, f"checkpoint_epoch_{epoch}.pt")
    torch.save({
        'epoch': epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
    }, checkpoint_path)
    print(f"Checkpoint saved: {checkpoint_path}")
