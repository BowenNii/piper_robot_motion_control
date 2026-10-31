"""输入文件摘要与来源记录。"""
from __future__ import annotations
import hashlib
from pathlib import Path

def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()
