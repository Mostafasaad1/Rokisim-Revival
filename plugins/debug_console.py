from rokisim_revival.plugin import Plugin
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QLineEdit,
    QPushButton, QComboBox, QLabel
)
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QTextCursor
import logging
import sys
import io
from datetime import datetime
from qfluentwidgets import FluentIcon

class DebugConsolePlugin(Plugin):
    name = "Debug Console"
    description = "Real-time logging and debugging console"
    version = "1.0.0"
    author = "RokiSim Team"

    def __init__(self):
        super().__init__()
        self.console_widget = None
        self.log_handler = None

    def create_ui_panel(self, parent) -> QWidget:
        if self.console_widget is None:
            self.console_widget = DebugConsoleWidget(parent)
            # Set up log capture
            self.log_handler = ConsoleLogHandler(self.console_widget)
            logging.getLogger().addHandler(self.log_handler)
            logging.getLogger().setLevel(logging.DEBUG)
        return self.console_widget

    def get_icon(self):
        return FluentIcon.SEARCH

    def get_ui_panel_title(self) -> str:
        return "Debug Console"

    def on_robot_loaded(self, robot_definition):
        self.console_widget.append_log(f"✅ Robot '{robot_definition.name}' loaded", "INFO")

    def on_program_compiled(self, joint_positions):
        self.console_widget.append_log(f"✅ Program compiled ({len(joint_positions)} steps)", "INFO")

    def execute(self, *args, **kwargs):
        return "Debug console is active in UI."

class DebugConsoleWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        # Title
        title = QLabel("🔍 Debug Console")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        # Log display
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setStyleSheet("""
            QTextEdit {
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 12px;
                background-color: #1e1e1e;
                color: #dcdcdc;
                border: 1px solid #444;
            }
        """)
        layout.addWidget(self.log_display, 1)

        # Controls
        ctrl_layout = QHBoxLayout()
        
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self._clear_logs)
        ctrl_layout.addWidget(self.clear_btn)

        self.save_btn = QPushButton("Save Log")
        self.save_btn.clicked.connect(self._save_log)
        ctrl_layout.addWidget(self.save_btn)

        ctrl_layout.addStretch()
        layout.addLayout(ctrl_layout)

    def append_log(self, message: str, level: str = "INFO"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        color = {
            "ERROR": "#ff5555",
            "WARNING": "#f1fa8c",
            "INFO": "#50fa7b",
            "DEBUG": "#bd93f9"
        }.get(level, "#dcdcdc")

        formatted = f'<span style="color:#888;">[{timestamp}]</span> ' \
                   f'<span style="color:{color};">[{level}]</span> ' \
                   f'{message}'

        self.log_display.append(formatted)
        self.log_display.moveCursor(QTextCursor.End)

    def _clear_logs(self):
        self.log_display.clear()

    def _save_log(self):
        from PySide6.QtWidgets import QFileDialog
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Log", "rokisim_debug.log", "Log Files (*.log);;Text Files (*.txt)"
        )
        if filename:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(self.log_display.toPlainText())
            self.append_log(f"Log saved to {filename}", "INFO")

class ConsoleLogHandler(logging.Handler):
    def __init__(self, console_widget):
        super().__init__()
        self.console_widget = console_widget

    def emit(self, record):
        msg = self.format(record)
        level = record.levelname
        # Emit to console widget (thread-safe via Qt signal if needed)
        try:
            self.console_widget.append_log(msg, level)
        except RuntimeError:
            # Widget destroyed
            pass