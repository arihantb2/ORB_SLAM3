"""Pytest configuration: add evals/ to sys.path so packages are importable."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
