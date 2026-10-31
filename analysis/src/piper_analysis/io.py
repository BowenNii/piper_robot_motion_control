"""兼容旧导入；实现位于data_io。"""
from __future__ import annotations
from .data_io.csv import read_numeric, columns, write_rows
from .data_io.feedback import feedback
from .data_io.config import load_config
from .data_io.provenance import sha256

