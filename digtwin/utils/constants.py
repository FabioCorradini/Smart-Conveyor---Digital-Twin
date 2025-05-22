import os
from pathlib import Path

DATA_DIR = Path(os.getenv("DATA_DIR", "./Data"))
MODELS_DIR = DATA_DIR / "models"

if not MODELS_DIR.is_dir():
    MODELS_DIR.mkdir()

NODES_DIR = DATA_DIR / "nodes"

if not NODES_DIR.is_dir():
    NODES_DIR.mkdir()

STATE_MODELS_DIR = DATA_DIR / "state-models"

if not STATE_MODELS_DIR.is_dir():
    STATE_MODELS_DIR.mkdir()