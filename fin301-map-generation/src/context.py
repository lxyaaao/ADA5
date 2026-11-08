"""Paths injected by the CLI; no machine-specific directories or dataset globals."""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATASET = os.environ.get('MAP_DATASET', 'fin301')
WORK = Path(os.environ.get('MAP_WORK_DIR', str(ROOT / 'work' / DATASET)))
BAG = Path(os.environ.get('MAP_BAG_PATH', str(ROOT / 'data' / DATASET / (DATASET + '_0.db3'))))
SOURCE = WORK / 'backend'
for directory in (SOURCE,):
    directory.mkdir(parents=True, exist_ok=True)
