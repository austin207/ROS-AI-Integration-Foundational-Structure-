#!/usr/bin/env python3
import os
import requests

def download_file(url, dest_folder):
    if not os.path.exists(dest_folder):
        os.makedirs(dest_folder)
    local_filename = url.split('/')[-1]
    local_filepath = os.path.join(dest_folder, local_filename)
    with requests.get(url, stream=True) as r:
        r.raise_for_status()
        with open(local_filepath, 'wb') as f:
            for chunk in r.iter_content(chunk_size=8192):
                f.write(chunk)
    return local_filepath

if __name__ == '__main__':
    url = "https://example.com/data.txt"  # Replace with your actual URL
    dest_folder = os.path.join(os.path.dirname(__file__), "../../data/raw")
    print(f"Downloading file from {url} to {dest_folder} ...")
    filepath = download_file(url, dest_folder)
    print(f"Downloaded file to {filepath}")
