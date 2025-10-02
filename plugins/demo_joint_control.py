# plugins/joint_control_demo.py

from rokisim_revival.plugin import Plugin
from PySide6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from qfluentwidgets import FluentIcon, PrimaryPushButton, CardWidget, BodyLabel

class JointControlDemoPlugin(Plugin):
    name = "Joint Control Demo"
    description = "Read and control robot joints safely"
    version = "1.0.0"
    author = "Your Name"

    def __init__(self):
        super().__init__()
        # self.panel = None

    def get_icon(self):
        return FluentIcon.ROBOT

    def get_ui_panel_title(self) -> str:
        return "Joint Control Demo"

    def create_ui_panel(self, parent: "JointControlGUI") -> QWidget:
        # ALWAYS create a fresh widget — no caching!
        return JointControlDemoWidget(parent)

    def execute(self, *args, **kwargs):
        """
        Called when user clicks 'Execute Selected' in Plugins tab.
        Example: args = ["move", "45"]
        """
        gui = kwargs.get('gui')  # Will be passed if you modify _execute_selected_plugin
        if gui is None:
            return "❌ GUI not available. Make sure JointControlGUI passes 'gui'."

        if not args:
            return "ℹ️ Usage: 'read', 'move,<angle>', or 'reset'"

        cmd = args[0].lower()
        if cmd == "read":
            angles = gui.get_joint_angles()
            return f"Current angles: {[round(a, 2) for a in angles]}°"

        elif cmd == "move":
            if len(args) < 2:
                return "❌ Usage: move,<angle>"
            try:
                angle = float(args[1])
                success = gui.set_joint_target(0, angle)
                return f"✅ Joint 1 → {angle}°" if success else "❌ Failed"
            except ValueError:
                return "❌ Invalid angle"

        elif cmd == "reset":
            zeros = [0.0] * len(gui.get_joint_angles())
            success = gui.set_all_joint_targets(zeros)
            return "✅ All joints reset" if success else "❌ Failed"

        return f"❌ Unknown command: {cmd}"

class JointControlDemoWidget(CardWidget):
    def __init__(self, parent: "JointControlGUI"):
        super().__init__()
        self.gui = parent  # ← parent IS the JointControlGUI
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        title = BodyLabel("Joint Manipulation Demo")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        # Read button
        btn_read = PrimaryPushButton("Read Current Angles")
        btn_read.clicked.connect(self._read_angles)
        layout.addWidget(btn_read)

        # Move Joint 1
        btn_move = PrimaryPushButton("Move Joint 1 to 45°")
        btn_move.clicked.connect(self._move_joint_1)
        layout.addWidget(btn_move)

        # Reset all
        btn_reset = PrimaryPushButton("Reset All Joints")
        btn_reset.clicked.connect(self._reset_all)
        layout.addWidget(btn_reset)

        # Status
        self.status = BodyLabel("Ready")
        self.status.setStyleSheet("color: #0078D4; margin-top: 10px;")
        layout.addWidget(self.status)

    def _read_angles(self):
        if self.gui is None:
            self.status.setText("❌ GUI not available")
            return
        try:
            angles = self.gui.get_joint_angles()
            if not angles:
                self.status.setText("⚠️ Load a robot first")
            else:
                self.status.setText(f"Current: {[round(a, 1) for a in angles]}°")
        except Exception as e:
            self.status.setText(f"❌ Error: {e}")
            
    def _move_joint_1(self):
        try:
            success = self.gui.set_joint_target(0, 45.0)
            self.status.setText("✅ Joint 1 → 45°" if success else "❌ Failed")
        except Exception as e:
            self.status.setText(f"❌ Error: {e}")

    def _reset_all(self):
        try:
            zeros = [0.0] * len(self.gui.get_joint_angles())
            success = self.gui.set_all_joint_targets(zeros)
            self.status.setText("✅ All reset" if success else "❌ Failed")
        except Exception as e:
            self.status.setText(f"❌ Error: {e}")