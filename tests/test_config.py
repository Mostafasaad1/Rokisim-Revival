import json
import os
import tempfile

import pytest
from src.rokisim_revival.config import Config


def test_config_defaults():
    config = Config()
    assert config.ip == "127.0.0.1"
    assert config.port == 2001
    assert config.interpolation_steps == 10


def test_load_config():
    data = {"ip": "192.168.1.1", "port": 8080, "interpolation_steps": 20}
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as tmp:
        json.dump(data, tmp)
    config = Config(tmp.name)
    assert config.ip == "192.168.1.1"
    assert config.port == 8080
    assert config.interpolation_steps == 20
    os.unlink(tmp.name)


def test_save_config():
    config = Config()
    config.ip = "192.168.1.1"
    with tempfile.NamedTemporaryFile(mode="r", suffix=".json", delete=False) as tmp:
        config.save(tmp.name)
        loaded = json.load(tmp)
    assert loaded["ip"] == "192.168.1.1"
    assert loaded["port"] == 2001
    os.unlink(tmp.name)
