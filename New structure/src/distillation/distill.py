#!/usr/bin/env python3
import torch
import torch.nn as nn
import torch.optim as optim
from llm_training.model import TransformerModel
from loss_functions import distillation_loss
import yaml
import os
from llm_training.utils import get_data_iterator, save_checkpoint

def distill_model(config):
    vocab_size = config['model']['vocab_size']
    # Teacher: full-scale LLM (assumed already trained)
    teacher = TransformerModel(
        vocab_size=vocab_size,
        d_model=config['teacher']['d_model'],
        nhead=config['teacher']['nhead'],
        num_layers=config['teacher']['num_layers'],
        dim_feedforward=config['teacher']['dim_feedforward'],
        dropout=config['teacher']['dropout']
    )
    # Student: smaller model to be distilled
    student = TransformerModel(
        vocab_size=vocab_size,
        d_model=config['student']['d_model'],
        nhead=config['student']['nhead'],
        num_layers=config['student']['num_layers'],
        dim_feedforward=config['student']['dim_feedforward'],
        dropout=config['student']['dropout']
    )

    optimizer = optim.Adam(student.parameters(), lr=config['distillation']['lr'])
    num_epochs = config['distillation']['epochs']

    data_iter = get_data_iterator(config['data']['train_file'], config['distillation']['batch_size'], vocab_size)
    student.train()
    teacher.eval()  # Freeze teacher parameters
    for epoch in range(1, num_epochs + 1):
        total_loss = 0.
        for batch, (src, targets) in enumerate(data_iter):
            optimizer.zero_grad()
            with torch.no_grad():
                teacher_output = teacher(src)
            student_output = student(src)
            loss = distillation_loss(student_output, teacher_output, targets)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
            if batch % config['distillation']['log_interval'] == 0 and batch > 0:
                print(f"Epoch {epoch}, Batch {batch}, Distillation Loss: {loss.item():.4f}")
            if batch >= config['distillation']['steps_per_epoch']:
                break
        avg_loss = total_loss / (batch + 1)
        print(f"Epoch {epoch} distillation completed. Average Loss: {avg_loss:.4f}")
        save_checkpoint(student, optimizer, epoch, config['distillation']['checkpoint_dir'])
    return student

if __name__ == '__main__':
    with open('config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    if not os.path.exists(config['distillation']['checkpoint_dir']):
        os.makedirs(config['distillation']['checkpoint_dir'])
    distilled_model = distill_model(config)
