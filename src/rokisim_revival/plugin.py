from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Callable
from PySide6.QtWidgets import QWidget
from qfluentwidgets import FluentIconBase, FluentIcon


class Plugin(ABC):
    name: str = "Unnamed Plugin"
    description: str = "No description provided."
    version: str = "0.1.0"
    author: str = "Unknown"

    # --- Core execution (unchanged)
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
    def get_icon(self) -> FluentIconBase:
        """Return the Fluent icon for this plugin's tab. Override in subclass."""
        return FluentIcon.APPLICATION  # Default fallback

    def create_ui_panel(self, parent: QWidget) -> Optional[QWidget]:
        """
        Return a custom QWidget to embed in the main GUI.
        Return None if no UI is provided.
        """
        return None

    def get_ui_panel_title(self) -> str:
        """Title for the plugin tab or card."""
        return f"{self.name} Panel"

    # --- NEW: Program Instruction Extension
    def register_instructions(self) -> Dict[str, Callable]:
        """
        Return a dict of {command_name: handler_function}.
        Handler signature: handler(parts: List[str], pc: int, lines: List[str]) -> int
        """
        return {}

    # --- NEW: Lifecycle Hooks (optional)
    def on_robot_loaded(self, robot_definition) -> None:
        pass

    def on_program_compiled(self, joint_positions) -> None:
        pass