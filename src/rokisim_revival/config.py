import json
import os
from typing import Optional


class Config:
    """Centralized configuration for the application."""

    def __init__(self, config_file: Optional[str] = None):
        self.ip: str = "127.0.0.1"
        self.port: int = 2001
        self.buffer_size: int = 56
        self.protocol_num_joints: int = 6
        self.interpolation_steps: int = 10
        self.log_level: str = "INFO"

        if config_file and os.path.exists(config_file):
            self._load_from_file(config_file)

    def _load_from_file(self, file_path: str) -> None:
        with open(file_path, "r") as f:
            data = json.load(f)
            self.ip = data.get("ip", self.ip)
            self.port = data.get("port", self.port)
            self.interpolation_steps = data.get(
                "interpolation_steps", self.interpolation_steps
            )
            # Add more as needed

    def save(self, file_path: str) -> None:
        data = {
            "ip": self.ip,
            "port": self.port,
            "interpolation_steps": self.interpolation_steps,
        }
        with open(file_path, "w") as f:
            json.dump(data, f)
