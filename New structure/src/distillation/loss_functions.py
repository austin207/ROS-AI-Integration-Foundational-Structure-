#!/usr/bin/env python3
import torch
import torch.nn.functional as F

def distillation_loss(student_output, teacher_output, true_targets, temperature=2.0, alpha=0.5):
    """
    Combines KL divergence on softened outputs and cross-entropy loss on true targets.
    """
    # Soft target loss: KL divergence between teacher and student distributions
    soft_loss = F.kl_div(
        F.log_softmax(student_output / temperature, dim=-1),
        F.softmax(teacher_output / temperature, dim=-1),
        reduction='batchmean'
    ) * (temperature ** 2)
    # Hard target loss: standard cross entropy with true targets
    hard_loss = F.cross_entropy(student_output.view(-1, student_output.size(-1)), true_targets.view(-1))
    return alpha * hard_loss + (1 - alpha) * soft_loss
