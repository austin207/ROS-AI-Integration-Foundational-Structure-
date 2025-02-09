#!/usr/bin/env python3
import logging

def setup_logger(name, log_file, level=logging.INFO):
    """Setup a logger."""
    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    handler = logging.FileHandler(log_file)
    handler.setFormatter(formatter)
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)
    return logger

def print_config(config):
    for key, value in config.items():
        print(f"{key}: {value}")
