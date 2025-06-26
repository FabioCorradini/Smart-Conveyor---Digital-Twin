import os
from pathlib import Path

DATA_DIR = Path(os.getenv("DATA_DIR", "./Data"))

if not DATA_DIR.is_dir():
    DATA_DIR.mkdir()

MODELS_DIR = DATA_DIR / "models"

if not MODELS_DIR.is_dir():
    MODELS_DIR.mkdir()

NODES_DIR = DATA_DIR / "nodes"

if not NODES_DIR.is_dir():
    NODES_DIR.mkdir()

STATE_MODELS_DIR = DATA_DIR / "state-models"

if not STATE_MODELS_DIR.is_dir():
    STATE_MODELS_DIR.mkdir()

ACTOR_DIR = DATA_DIR / "actors"

if not ACTOR_DIR.is_dir():
    ACTOR_DIR.mkdir()

SENSORS_DIR = DATA_DIR / "sensors"

if not SENSORS_DIR.is_dir():
    SENSORS_DIR.mkdir()

QT_NODES_DIR = DATA_DIR / "qt-nodes"

if not QT_NODES_DIR.is_dir():
    QT_NODES_DIR.mkdir()

DIGITAL_TWIN_MQTT_ADDRESS = "localhost"
DIGITAL_TWIN_MQTT_PORT = 1883