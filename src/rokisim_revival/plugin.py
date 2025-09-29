# plugin.py
from abc import ABC, abstractmethod
from typing import Any, Dict

class Plugin(ABC):
    name: str = "Unnamed Plugin"
    description: str = "No description provided."
    version: str = "0.1.0"
    author: str = "Unknown"

    @abstractmethod
    def execute(self, *args, **kwargs) -> Any:
        raise NotImplementedError

    def validate(self) -> bool:
        return True

    def get_metadata(self) -> Dict[str, str]:
        return {
            "name": self.name,
            "description": self.description,
            "version": self.version,
            "author": self.author,
        }