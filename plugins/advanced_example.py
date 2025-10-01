from rokisim_revival.plugin import Plugin
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from typing import List

class AdvancedExamplePlugin(Plugin):
    name = "Advanced Example"
    description = "Adds custom command and UI panel"
    version = "1.0.0"
    author = "You"

    def execute(self, *args, **kwargs):
        return "This plugin extends the app!"

    def create_ui_panel(self, parent) -> QWidget:
        panel = QWidget(parent)
        layout = QVBoxLayout(panel)
        layout.addWidget(QLabel("🎉 This is a custom plugin panel!"))
        btn = QPushButton("Do Something")
        btn.clicked.connect(lambda: print("Custom action triggered!"))
        layout.addWidget(btn)
        return panel

    def get_ui_panel_title(self) -> str:
        return "My Addon"

    def register_instructions(self):
        return {
            "ADDON": self._handle_addon
        }

    def _handle_addon(self, parts: List[str], pc: int, lines: List[str]) -> int:
        # Example: ADDON SET_ANGLE 3 45.0
        if len(parts) < 4:
            raise ValueError("ADDON requires: command joint_id angle")
        cmd = parts[1]
        if cmd == "SET_ANGLE":
            joint_id = int(parts[2]) - 1  # 1-based in program
            angle = float(parts[3])
            # Inject into current pose (requires access to compiler state)
            if hasattr(self, '_compiler_ref'):
                if self._compiler_ref.current_pose is not None:
                    self._compiler_ref.current_pose[joint_id] = angle
        return pc + 1