import logging
import math
import os
import sys
import threading
import time
from typing import List, Optional, Tuple

from PySide6.QtCore import QObject, QPoint, QRect, Qt, Signal, QTimer, QEasingCurve
from PySide6.QtGui import QColor, QDoubleValidator, QFont, QPainter, QPen, QLinearGradient, QRadialGradient
from PySide6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
    QGridLayout,
    QGraphicsDropShadowEffect,
    QSizePolicy
)

from qfluentwidgets import (
    setTheme, Theme, PushButton, PrimaryPushButton, Slider, PlainTextEdit, 
    LineEdit, DoubleSpinBox, SmoothScrollArea, BodyLabel, InfoBar, 
    InfoBarPosition, Pivot, isDarkTheme, FluentWindow, FluentIcon, 
    CardWidget, TitleLabel, SubtitleLabel, StrongBodyLabel, 
    ToolButton, ToggleButton, SwitchButton, ComboBox, 
    NavigationItemPosition, MessageBox, TeachingTip, 
    TeachingTipTailPosition, setThemeColor
)

from .instruction_set import InstructionSetCompiler
from .sender import RoKiSimSender


class FocusAwareDoubleSpinBox(DoubleSpinBox):
    """A custom spin box that emits signals when it gains or loses focus."""
    
    focused_in = Signal()
    focused_out = Signal()
    
    def focusInEvent(self, event):
        self.focused_in.emit()
        super().focusInEvent(event)
    
    def focusOutEvent(self, event):
        self.focused_out.emit()
        super().focusOutEvent(event)


class SpeedGauge(QWidget):
    """Modern minimalist speed gauge with smooth animations and elegant design."""

    def __init__(self, max_speed: float = 50.0, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.max_speed = max_speed
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.joint_name = "Joint"
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animate_speed)
        self.animation_timer.setInterval(16)  # ~60 FPS
        self.animation_duration = 250  # ms
        self.animation_steps = 0
        self.total_steps = 0
        
        # Set fixed size for consistent appearance
        self.setFixedSize(140, 140)

    def set_speed(self, speed: float) -> None:
        self.target_speed = min(abs(speed), self.max_speed)
        if not self.animation_timer.isActive():
            self.animation_steps = 0
            self.total_steps = int(self.animation_duration / 16)
            self.animation_timer.start()

    def reset_speed(self) -> None:
        """Reset speed to zero"""
        self.set_speed(0.0)

    def set_joint_name(self, name: str) -> None:
        self.joint_name = name
        self.update()

    def _animate_speed(self):
        """Smoothly animate the speed value"""
        if self.animation_steps < self.total_steps:
            # Ease-out cubic interpolation
            t = self.animation_steps / self.total_steps
            eased_t = 1 - pow(1 - t, 3)
            self.current_speed = self.current_speed + (self.target_speed - self.current_speed) * eased_t
            self.animation_steps += 1
            self.update()
        else:
            self.current_speed = self.target_speed
            self.animation_timer.stop()
            self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        size = min(self.width(), self.height()) - 20
        center = QPoint(self.width() // 2, self.height() // 2)
        radius = size // 2
        
        # Theme-aware colors
        if isDarkTheme():
            # Dark theme colors
            bg_color = QColor(30, 30, 30)
            track_color = QColor(60, 60, 60)
            text_color = QColor(220, 220, 220)
            highlight_color = QColor(0, 120, 212)  # Fluent blue
            progress_start = QColor(0, 150, 136)   # Teal
            progress_end = QColor(0, 120, 212)     # Blue
        else:
            # Light theme colors
            bg_color = QColor(250, 250, 250)
            track_color = QColor(220, 220, 220)
            text_color = QColor(40, 40, 40)
            highlight_color = QColor(0, 120, 212)  # Fluent blue
            progress_start = QColor(0, 150, 136)   # Teal
            progress_end = QColor(0, 120, 212)     # Blue
        
        # Draw background circle
        painter.setPen(Qt.NoPen)
        painter.setBrush(bg_color)
        painter.drawEllipse(center, radius, radius)
        
        # Draw track
        painter.setPen(QPen(track_color, 4))
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(
            center.x() - radius, center.y() - radius,
            radius * 2, radius * 2,
            -90 * 16, 180 * 16
        )
        
        # Draw progress arc
        if self.max_speed > 0 and self.current_speed > 0:
            progress_angle = int((self.current_speed / self.max_speed) * 180)
            
            # Create smooth gradient
            gradient = QLinearGradient(
                center.x() - radius, center.y(),
                center.x() + radius, center.y()
            )
            gradient.setColorAt(0, progress_start)
            gradient.setColorAt(1, progress_end)
            
            painter.setPen(QPen(gradient, 6))
            painter.drawArc(
                center.x() - radius, center.y() - radius,
                radius * 2, radius * 2,
                -90 * 16, progress_angle * 16
            )
        
        # Draw center circle
        painter.setPen(Qt.NoPen)
        painter.setBrush(bg_color)
        painter.drawEllipse(center, radius - 15, radius - 15)
        
        # Draw speed value - LARGE and prominent
        painter.setPen(highlight_color)
        painter.setFont(QFont("Segoe UI", 16, QFont.DemiBold))
        speed_text = f"{self.current_speed:.1f}"
        speed_rect = QRect(center.x() - 40, center.y() - 15, 80, 35)
        painter.drawText(speed_rect, Qt.AlignCenter, speed_text)
        
        # Draw unit label
        painter.setPen(text_color)
        painter.setFont(QFont("Segoe UI", 9))
        unit_rect = QRect(center.x() - 40, center.y() + 18, 80, 20)
        painter.drawText(unit_rect, Qt.AlignCenter, "°/s")
        
        # Draw joint name at top
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        name_rect = QRect(center.x() - 50, center.y() - radius + 5, 100, 25)
        painter.drawText(name_rect, Qt.AlignCenter, self.joint_name)
        
        # Draw minimal tick marks
        painter.setPen(QPen(text_color, 1))
        for i in range(0, 181, 45):  # Only major ticks
            angle = math.radians(i - 90)
            inner_x = center.x() + (radius - 12) * math.cos(angle)
            inner_y = center.y() + (radius - 12) * math.sin(angle)
            outer_x = center.x() + (radius - 5) * math.cos(angle)
            outer_y = center.y() + (radius - 5) * math.sin(angle)
            painter.drawLine(QPoint(int(inner_x), int(inner_y)), QPoint(int(outer_x), int(outer_y)))
            
            # Draw value labels
            if i % 90 == 0:  # Only at 0, 90, 180
                value = int((i / 180.0) * self.max_speed)
                label_x = center.x() + (radius - 25) * math.cos(angle)
                label_y = center.y() + (radius - 25) * math.sin(angle)
                painter.setFont(QFont("Segoe UI", 8))
                label_rect = QRect(int(label_x) - 10, int(label_y) - 8, 20, 16)
                painter.drawText(label_rect, Qt.AlignCenter, str(value))


class Worker(QObject):
    update_signal = Signal(list)
    fk_update_signal = Signal(tuple, tuple)
    speed_update_signal = Signal(list)  # New signal for speed updates

    def __init__(self, sender: RoKiSimSender, num_joints: int, max_speeds: List[float]):
        super().__init__()
        self.sender = sender
        self.num_joints = num_joints
        self.max_speeds = max_speeds
        self.running = True
        self.joint_angles: List[float] = [0.0] * num_joints
        self.target_angles: List[float] = [0.0] * num_joints
        self.last_sent_angles: List[float] = [0.0] * num_joints
        self.previous_angles: List[float] = [0.0] * num_joints  # For speed calculation
        self.UPDATE_INTERVAL = 0.05
        self.SEND_THRESHOLD = 0.1
        self.global_speed_factor = 1.0
        
        # Add debouncing for slider updates
        self.last_update_time = time.time()
        self.MIN_UPDATE_INTERVAL = 0.1  # Minimum 100ms between updates

    def update_target_angle(self, index: int, value: float) -> None:
        self.target_angles[index] = value

    def set_global_speed_factor(self, factor: float) -> None:
        self.global_speed_factor = factor

    def _interpolate_angle(
        self, current: float, target: float, max_speed: float, dt: float
    ) -> float:
        delta = target - current
        max_step = max_speed * dt * self.global_speed_factor
        if abs(delta) <= max_step:
            return target
        return current + (max_step if delta > 0 else -max_step)

    def run(self) -> None:
        while self.running:
            current_time = time.time()
            has_change = False
            
            # Only process updates at appropriate intervals to reduce jitter
            if current_time - self.last_update_time >= self.MIN_UPDATE_INTERVAL:
                # Store previous angles for speed calculation
                self.previous_angles = self.joint_angles[:]
                
                for i in range(self.num_joints):
                    new_angle = self._interpolate_angle(
                        self.joint_angles[i],
                        self.target_angles[i],
                        self.max_speeds[i],
                        self.UPDATE_INTERVAL,
                    )
                    if new_angle != self.joint_angles[i]:
                        self.joint_angles[i] = new_angle
                        if (
                            abs(self.joint_angles[i] - self.last_sent_angles[i])
                            > self.SEND_THRESHOLD
                        ):
                            has_change = True
                
                if has_change and self.num_joints > 0:
                    try:
                        self.sender.send_angles(self.joint_angles)
                        self.last_sent_angles[:] = self.joint_angles
                        self.last_update_time = current_time
                        self.update_signal.emit(self.joint_angles[:])
                        pos, rpy = self.sender.calculate_fk(self.joint_angles)
                        if pos and rpy:
                            self.fk_update_signal.emit(pos, rpy)
                    except Exception as e:
                        logging.error(f"Error in worker: {e}")
                
                # Calculate and emit speeds
                speeds = []
                for i in range(self.num_joints):
                    # Calculate speed as angle change per second
                    angle_change = abs(self.joint_angles[i] - self.previous_angles[i])
                    speed = angle_change / self.UPDATE_INTERVAL if self.UPDATE_INTERVAL > 0 else 0.0
                    speeds.append(speed)
                self.speed_update_signal.emit(speeds)
            
            time.sleep(self.UPDATE_INTERVAL)

    def stop(self) -> None:
        self.running = False
        # Emit zero speeds when stopping
        self.speed_update_signal.emit([0.0] * self.num_joints)


class JointControlGUI(FluentWindow):
    def __init__(self, sender: RoKiSimSender):
        super().__init__()
        self.sender = sender
        self.worker: Optional[Worker] = None
        self.worker_thread: Optional[threading.Thread] = None
        self.compiled_program: Optional[List[List[float]]] = None
        self.program_running = False
        self.program_thread: Optional[threading.Thread] = None
        self.compiler = InstructionSetCompiler()
        self.program_step_delay = 0.5  # Default delay
        
        # Add control state tracking
        self.user_interaction_active = False
        self.joint_edit_flags = []  # Track which joints are being edited by user
        self.slider_update_timers = []  # Timers for debouncing slider updates
        self.reset_in_progress = False
        
        # Set window properties
        self.setWindowTitle("RoKiSim Robot Controller")
        self.resize(1200, 800)
        
        # Create interfaces
        self.joint_tab = QWidget()
        self.joint_tab.setObjectName("jointTab")
        self.program_tab = QWidget()
        self.program_tab.setObjectName("programTab")
        
        self._setup_joint_tab()
        self._setup_program_tab()
        
        # Add interfaces to navigation
        self.addSubInterface(self.joint_tab, FluentIcon.ROBOT, 'Joint Control')
        self.addSubInterface(self.program_tab, FluentIcon.CODE, 'Program Editor')
        
        # Set theme and colors
        setTheme(Theme.AUTO)
        setThemeColor('#0078D4')

    def _setup_joint_tab(self) -> None:
        layout = QVBoxLayout(self.joint_tab)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(25)
        
        # Title
        title_label = TitleLabel("Robot Joint Control")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # Load button
        self.load_button = PrimaryPushButton(FluentIcon.FOLDER, "Load Robot Definition")
        self.load_button.setFixedHeight(45)
        self.load_button.clicked.connect(self.load_robot_xml)
        layout.addWidget(self.load_button)
        
        # Main content splitter
        content_widget = QWidget()
        content_layout = QHBoxLayout(content_widget)
        content_layout.setSpacing(25)
        
        # Left panel - Joint controls
        left_panel = CardWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(20, 20, 20, 20)
        left_layout.setSpacing(15)
        
        # Joint controls title
        joint_title = SubtitleLabel("Joint Controls")
        left_layout.addWidget(joint_title)
        
        # Joint controls scroll area
        self.joint_controls_card = CardWidget()
        self.joint_controls_layout = QVBoxLayout(self.joint_controls_card)
        self.joint_controls_layout.setContentsMargins(15, 15, 15, 15)
        self.joint_controls_layout.setSpacing(12)
        
        scroll = SmoothScrollArea()
        scroll.setWidget(self.joint_controls_card)
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("SmoothScrollArea { border: none; }")
        left_layout.addWidget(scroll)
        
        content_layout.addWidget(left_panel)
        
        # Right panel - FK/IK controls
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(25)
        
        # Speed control
        speed_card = CardWidget()
        speed_layout = QVBoxLayout(speed_card)
        speed_layout.setContentsMargins(20, 15, 20, 15)
        speed_layout.setSpacing(12)
        
        speed_title = SubtitleLabel("Global Speed Control")
        speed_layout.addWidget(speed_title)
        
        speed_factor_layout = QHBoxLayout()
        speed_factor_layout.setSpacing(15)
        speed_factor_layout.addWidget(BodyLabel("Speed Factor:"))
        self.speed_factor_slider = Slider(Qt.Horizontal)
        self.speed_factor_slider.setFixedHeight(24)
        self.speed_factor_slider.setRange(1, 20)
        self.speed_factor_slider.setValue(10)
        self.speed_factor_slider.valueChanged.connect(self.update_global_speed)
        speed_factor_layout.addWidget(self.speed_factor_slider)
        self.speed_factor_label = BodyLabel("1.0x")
        self.speed_factor_label.setFixedWidth(40)
        speed_factor_layout.addWidget(self.speed_factor_label)
        speed_layout.addLayout(speed_factor_layout)
        
        right_layout.addWidget(speed_card)
        
        # Forward Kinematics
        fk_card = CardWidget()
        fk_layout = QGridLayout(fk_card)
        fk_layout.setContentsMargins(20, 15, 20, 15)
        fk_layout.setSpacing(15)
        
        fk_title = SubtitleLabel("Forward Kinematics")
        fk_layout.addWidget(fk_title, 0, 0, 1, 2)
        
        self.fk_labels = [StrongBodyLabel() for _ in range(6)]
        fk_labels_text = ["X:", "Y:", "Z:", "Roll:", "Pitch:", "Yaw:"]
        for i in range(6):
            row = (i // 2) + 1
            col = (i % 2) * 2
            fk_layout.addWidget(BodyLabel(fk_labels_text[i]), row, col)
            fk_layout.addWidget(self.fk_labels[i], row, col + 1)
        right_layout.addWidget(fk_card)
        
        # Inverse Kinematics
        ik_card = CardWidget()
        ik_layout = QGridLayout(ik_card)
        ik_layout.setContentsMargins(20, 15, 20, 15)
        ik_layout.setSpacing(15)
        
        ik_title = SubtitleLabel("Inverse Kinematics")
        ik_layout.addWidget(ik_title, 0, 0, 1, 2)
        
        self.ik_entries = [LineEdit() for _ in range(6)]
        ik_labels_text = ["X:", "Y:", "Z:", "Roll:", "Pitch:", "Yaw:"]
        for i, entry in enumerate(self.ik_entries):
            row = (i // 2) + 1
            col = (i % 2) * 2
            ik_layout.addWidget(BodyLabel(ik_labels_text[i]), row, col)
            entry.setValidator(QDoubleValidator())
            entry.setFixedHeight(35)
            ik_layout.addWidget(entry, row, col + 1)
        
        self.ik_button = PrimaryPushButton(FluentIcon.MOVE, "Move to Pose (IK)")
        self.ik_button.setFixedHeight(35)
        self.ik_button.clicked.connect(self.move_to_ik_pose)
        ik_layout.addWidget(self.ik_button, 4, 0, 1, 2)
        
        self.ik_status_label = BodyLabel("")
        self.ik_status_label.setAlignment(Qt.AlignCenter)
        ik_layout.addWidget(self.ik_status_label, 5, 0, 1, 2)
        
        right_layout.addWidget(ik_card)
        
        content_layout.addWidget(right_panel)
        layout.addWidget(content_widget)

    def _setup_program_tab(self) -> None:
        layout = QVBoxLayout(self.program_tab)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(25)
        
        # Title
        title_label = TitleLabel("Robot Program Editor")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # Program editor section
        editor_card = CardWidget()
        editor_layout = QVBoxLayout(editor_card)
        editor_layout.setContentsMargins(20, 15, 20, 15)
        editor_layout.setSpacing(15)
        
        editor_title = SubtitleLabel("Program Editor")
        editor_layout.addWidget(editor_title)
        
        self.program_editor = PlainTextEdit()
        self.program_editor.setPlaceholderText(
            "# Enter your robot program here\n"
            "# Example commands:\n"
            "# MOVE J1 45\n"
            "# MOVE J2 30\n"
            "# DELAY 1000\n"
            "# HOME"
        )
        self.program_editor.setStyleSheet("font-size: 13px;")
        editor_layout.addWidget(self.program_editor)
        
        # Control buttons
        button_layout = QHBoxLayout()
        button_layout.setSpacing(15)
        
        self.compile_button = PrimaryPushButton(FluentIcon.COMMAND_PROMPT, "Compile")
        self.compile_button.setFixedHeight(35)
        self.compile_button.clicked.connect(self.compile_program)
        button_layout.addWidget(self.compile_button)
        
        self.run_button = PrimaryPushButton(FluentIcon.PLAY, "Run")
        self.run_button.setFixedHeight(35)
        self.run_button.clicked.connect(self.run_program)
        self.run_button.setEnabled(False)
        button_layout.addWidget(self.run_button)
        
        self.stop_button = PushButton(FluentIcon.CLOSE, "Stop")
        self.stop_button.setFixedHeight(35)
        self.stop_button.clicked.connect(self.stop_program)
        self.stop_button.setEnabled(False)
        button_layout.addWidget(self.stop_button)
        
        button_layout.addStretch()
        editor_layout.addLayout(button_layout)
        
        layout.addWidget(editor_card)
        
        # Output section
        output_card = CardWidget()
        output_layout = QVBoxLayout(output_card)
        output_layout.setContentsMargins(20, 15, 20, 15)
        output_layout.setSpacing(15)
        
        output_title = SubtitleLabel("Program Output")
        output_layout.addWidget(output_title)
        
        self.program_status_label = BodyLabel("")
        self.program_status_label.setAlignment(Qt.AlignCenter)
        output_layout.addWidget(self.program_status_label)
        
        self.output_display = PlainTextEdit()
        self.output_display.setReadOnly(True)
        self.output_display.setStyleSheet("font-size: 13px;")
        output_layout.addWidget(self.output_display)
        
        layout.addWidget(output_card)

    def update_global_speed(self, value: int) -> None:
        """Update global speed factor"""
        factor = value / 10.0
        self.speed_factor_label.setText(f"{factor:.1f}x")
        if self.worker:
            self.worker.set_global_speed_factor(factor)

    def load_robot_xml(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Load Robot XML", "", "XML Files (*.xml)"
        )
        if file_path:
            try:
                self.sender.load_robot_definition(file_path)
                num_joints = self.sender.robot_definition.get_num_joints()
                if num_joints == 0:
                    raise ValueError("No joints loaded from XML.")
                self._create_joint_controls(num_joints)
                self._start_worker(num_joints)
                InfoBar.success(
                    title="Success",
                    content=f"Loaded robot with {num_joints} joints.",
                    orient=Qt.Horizontal,
                    isClosable=True,
                    position=InfoBarPosition.TOP_RIGHT,
                    duration=3000,
                    parent=self
                )
            except Exception as e:
                InfoBar.error(
                    title="Error",
                    content=f"Failed to load XML: {e}",
                    orient=Qt.Horizontal,
                    isClosable=True,
                    position=InfoBarPosition.TOP_RIGHT,
                    duration=5000,
                    parent=self
                )

    def _create_joint_controls(self, num_joints: int) -> None:
        # Clear existing controls and state
        for i in reversed(range(self.joint_controls_layout.count())):
            widget = self.joint_controls_layout.itemAt(i).widget()
            if widget:
                widget.deleteLater()
        
        self.sliders: List[Slider] = []
        self.spin_boxes: List[FocusAwareDoubleSpinBox] = []
        self.gauges: List[SpeedGauge] = []
        self.reset_buttons: List[PushButton] = []
        self.joint_edit_flags = [False] * num_joints
        self.slider_update_timers = [None] * num_joints
        
        min_angles, max_angles = self.sender.robot_definition.get_joint_limits()
        
        for i in range(num_joints):
            joint_row = CardWidget()
            joint_row_layout = QHBoxLayout(joint_row)
            joint_row_layout.setContentsMargins(15, 12, 15, 12)
            joint_row_layout.setSpacing(20)
            
            # Joint info
            info_layout = QVBoxLayout()
            info_layout.setSpacing(4)
            
            joint_label = StrongBodyLabel(f"Joint {i+1}")
            info_layout.addWidget(joint_label)
            
            range_label = BodyLabel(f"[{min_angles[i]:.1f}°, {max_angles[i]:.1f}°]")
            range_label.setStyleSheet("font-size: 11px; color: #888888;")
            info_layout.addWidget(range_label)
            
            info_container = QWidget()
            info_container.setLayout(info_layout)
            info_container.setFixedWidth(120)
            joint_row_layout.addWidget(info_container)
            
            # Slider
            slider = Slider(Qt.Horizontal)
            slider.setRange(int(min_angles[i] * 10), int(max_angles[i] * 10))
            slider.setValue(0)
            slider.setFixedHeight(26)
            # Use proper QSizePolicy constants
            slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            
            # Connect slider signals with debouncing
            slider.sliderPressed.connect(lambda checked=False, idx=i: self.on_slider_pressed(idx))
            slider.sliderReleased.connect(lambda checked=False, idx=i: self.on_slider_released(idx))
            slider.valueChanged.connect(lambda v, idx=i: self.on_slider_value_changed(idx, v))
            
            self.sliders.append(slider)
            joint_row_layout.addWidget(slider, 1)  # Stretch factor = 1
            
            # Spin box
            spin_box = FocusAwareDoubleSpinBox()
            spin_box.setRange(min_angles[i], max_angles[i])
            spin_box.setValue(0.0)
            spin_box.setFixedWidth(100)
            spin_box.setFixedHeight(35)
            spin_box.setSuffix("°")
            spin_box.setStyleSheet("font-size: 13px;")
            
            # Connect spin box signals with edit tracking
            spin_box.focused_in.connect(lambda idx=i: self.on_spin_box_focus_in(idx))
            spin_box.focused_out.connect(lambda idx=i: self.on_spin_box_focus_out(idx))
            spin_box.valueChanged.connect(lambda v, idx=i: self.update_target_from_spin(idx, v))
            
            self.spin_boxes.append(spin_box)
            joint_row_layout.addWidget(spin_box)
            
            # Reset button
            reset_button = PushButton(FluentIcon.RETURN, "Reset")
            reset_button.setFixedWidth(80)
            reset_button.setFixedHeight(35)
            reset_button.clicked.connect(lambda _, idx=i: self.reset_joint(idx))
            self.reset_buttons.append(reset_button)
            joint_row_layout.addWidget(reset_button)
            
            # Speed gauge
            gauge = SpeedGauge()
            gauge.set_joint_name(f"J{i+1}")
            self.gauges.append(gauge)
            joint_row_layout.addWidget(gauge)
            
            self.joint_controls_layout.addWidget(joint_row)
        
        self.joint_controls_layout.addStretch()

    def on_slider_pressed(self, index: int) -> None:
        """Handle slider press - mark joint as being edited by user"""
        self.joint_edit_flags[index] = True
        self.user_interaction_active = True

    def on_slider_released(self, index: int) -> None:
        """Handle slider release - send final update and clear edit flag"""
        if self.slider_update_timers[index] is not None:
            self.slider_update_timers[index].stop()
        
        # Send final update immediately
        value = self.sliders[index].value() / 10.0
        if self.worker:
            self.worker.update_target_angle(index, value)
            self.spin_boxes[index].setValue(value)
        
        self.joint_edit_flags[index] = False
        self.user_interaction_active = any(self.joint_edit_flags)

    def on_slider_value_changed(self, index: int, value: int) -> None:
        """Handle slider value changes with debouncing"""
        if not self.joint_edit_flags[index]:
            return
            
        # Cancel any pending update for this slider
        if self.slider_update_timers[index] is not None:
            self.slider_update_timers[index].stop()
        
        # Create a new timer for debounced update
        timer = QTimer()
        timer.setSingleShot(True)
        timer.timeout.connect(lambda: self.debounced_slider_update(index, value))
        timer.start(50)  # 50ms debounce delay
        self.slider_update_timers[index] = timer
        
        # Update spin box immediately for responsive UI
        self.spin_boxes[index].setValue(value / 10.0)

    def debounced_slider_update(self, index: int, value: int) -> None:
        """Send debounced slider update to worker"""
        if self.worker and self.joint_edit_flags[index]:
            self.worker.update_target_angle(index, value / 10.0)

    def on_spin_box_focus_in(self, index: int) -> None:
        """Handle spin box focus in - mark as being edited"""
        self.joint_edit_flags[index] = True
        self.user_interaction_active = True

    def on_spin_box_focus_out(self, index: int) -> None:
        """Handle spin box focus out - clear edit flag"""
        self.joint_edit_flags[index] = False
        self.user_interaction_active = any(self.joint_edit_flags)

    def update_target_from_spin(self, index: int, value: float) -> None:
        """Update target from spin box - only if user is editing"""
        if self.joint_edit_flags[index] and self.worker:
            self.worker.update_target_angle(index, value)
            self.sliders[index].setValue(int(value * 10))

    def reset_joint(self, index: int) -> None:
        """Reset joint with conflict prevention"""
        if self.worker:
            # Set reset flag to prevent conflicts
            self.reset_in_progress = True
            
            # Update target angle
            self.worker.update_target_angle(index, 0.0)
            
            # Update UI immediately
            self.spin_boxes[index].setValue(0.0)
            self.sliders[index].setValue(0)
            
            # Clear reset flag after a short delay to allow UI updates to complete
            QTimer.singleShot(100, lambda: setattr(self, 'reset_in_progress', False))

    def _start_worker(self, num_joints: int) -> None:
        max_speeds = [50.0] * num_joints
        self.worker = Worker(self.sender, num_joints, max_speeds)
        self.worker.update_signal.connect(self.update_gui_from_worker)
        self.worker.fk_update_signal.connect(self.update_fk_display)
        self.worker.speed_update_signal.connect(self.update_speed_gauges)
        self.worker_thread = threading.Thread(target=self.worker.run, daemon=True)
        self.worker_thread.start()

    def update_gui_from_worker(self, angles: List[float]) -> None:
        """Update GUI from worker thread with conflict prevention"""
        if self.reset_in_progress:
            return
            
        for i, angle in enumerate(angles):
            # Only update controls if user is not actively editing them
            if not self.joint_edit_flags[i]:
                # Block signals temporarily to prevent feedback loops
                self.sliders[i].blockSignals(True)
                self.spin_boxes[i].blockSignals(True)
                
                self.sliders[i].setValue(int(angle * 10))
                self.spin_boxes[i].setValue(angle)
                
                # Restore signals
                self.sliders[i].blockSignals(False)
                self.spin_boxes[i].blockSignals(False)

    def update_speed_gauges(self, speeds: List[float]) -> None:
        """Update speed gauges with calculated speeds"""
        for i, speed in enumerate(speeds):
            self.gauges[i].set_speed(speed)

    def update_fk_display(self, pos: Tuple[float, ...], rpy: Tuple[float, ...]) -> None:
        """Update FK display - no conflicts with user input"""
        values = list(pos) + list(rpy)
        for i, label in enumerate(self.fk_labels):
            label.setText(f"{values[i]:.2f}")

    def move_to_ik_pose(self) -> None:
        """Move to IK pose with edit flag handling"""
        try:
            if any(not entry.text().strip() for entry in self.ik_entries):
                raise ValueError("Empty field")

            target_values = [float(entry.text()) for entry in self.ik_entries]
            target_pose = tuple(target_values)
            ik_solution = self.sender.calculate_ik(
                target_pose, self.worker.joint_angles if self.worker else None
            )
            if ik_solution:
                # Temporarily disable edit flags to allow programmatic updates
                original_flags = self.joint_edit_flags.copy()
                self.joint_edit_flags = [False] * len(self.joint_edit_flags)
                
                for i, angle in enumerate(ik_solution):
                    self.update_target_from_spin(i, angle)
                
                # Restore original flags after update
                self.joint_edit_flags = original_flags
                
                self.ik_status_label.setText("IK Success")
                self.ik_status_label.setStyleSheet("color: #107C10;")
                InfoBar.success(
                    title="Success",
                    content="Inverse kinematics solution found!",
                    orient=Qt.Horizontal,
                    isClosable=True,
                    position=InfoBarPosition.TOP_RIGHT,
                    duration=2000,
                    parent=self
                )
            else:
                self.ik_status_label.setText("IK Failed")
                self.ik_status_label.setStyleSheet("color: #A4262C;")
                InfoBar.error(
                    title="Error",
                    content="Inverse kinematics failed to find a solution",
                    orient=Qt.Horizontal,
                    isClosable=True,
                    position=InfoBarPosition.TOP_RIGHT,
                    duration=3000,
                    parent=self
                )
        except ValueError as e:
            self.ik_status_label.setText(f"Error: {e}")
            self.ik_status_label.setStyleSheet("color: #A4262C;")
            InfoBar.error(
                title="Input Error",
                content=str(e),
                orient=Qt.Horizontal,
                isClosable=True,
                position=InfoBarPosition.TOP_RIGHT,
                duration=3000,
                parent=self
            )

    def compile_program(self) -> None:
        program_text = self.program_editor.toPlainText()
        if not program_text.strip():
            self.program_status_label.setText("No program")
            self.program_status_label.setStyleSheet("color: #A4262C;")
            InfoBar.warning(
                title="Warning",
                content="Program editor is empty",
                orient=Qt.Horizontal,
                isClosable=True,
                position=InfoBarPosition.TOP_RIGHT,
                duration=2000,
                parent=self
            )
            return
        try:
            self.compiler = InstructionSetCompiler(
                self.compiler.default_interpolation_steps
            )
            if self.worker:
                self.compiler.current_pose = self.worker.joint_angles[:]
            self.compiled_program = self.compiler.compile(program_text)
            self.program_status_label.setText(
                f"Compiled: {len(self.compiled_program)} positions"
            )
            self.program_status_label.setStyleSheet("color: #107C10;")
            self.run_button.setEnabled(True)
            self.output_display.setPlainText(
                "Compiled positions:\n"
                + "\n".join(
                    f"{i}: {pos}" for i, pos in enumerate(self.compiled_program)
                )
            )
            InfoBar.success(
                title="Success",
                content=f"Compiled {len(self.compiled_program)} positions",
                orient=Qt.Horizontal,
                isClosable=True,
                position=InfoBarPosition.TOP_RIGHT,
                duration=2000,
                parent=self
            )
        except Exception as e:
            self.program_status_label.setText(f"Error: {e}")
            self.program_status_label.setStyleSheet("color: #A4262C;")
            self.run_button.setEnabled(False)
            InfoBar.error(
                title="Compilation Error",
                content=str(e),
                orient=Qt.Horizontal,
                isClosable=True,
                position=InfoBarPosition.TOP_RIGHT,
                duration=5000,
                parent=self
            )

    def run_program(self) -> None:
        if not self.compiled_program or not self.worker:
            self.program_status_label.setText("No program or robot")
            self.program_status_label.setStyleSheet("color: #A4262C;")
            InfoBar.error(
                title="Error",
                content="No program or robot loaded",
                orient=Qt.Horizontal,
                isClosable=True,
                position=InfoBarPosition.TOP_RIGHT,
                duration=3000,
                parent=self
            )
            return
        self.program_running = True
        self.compile_button.setEnabled(False)
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.program_status_label.setText("Running...")
        self.program_status_label.setStyleSheet("color: #0078D4;")
        self.program_thread = threading.Thread(
            target=self._execute_program, daemon=True
        )
        self.program_thread.start()

    def stop_program(self) -> None:
        self.program_running = False
        if self.program_thread and self.program_thread.is_alive():
            self.program_thread.join(timeout=1.0)
        self.compile_button.setEnabled(True)
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.program_status_label.setText("Stopped")
        self.program_status_label.setStyleSheet("color: #D83B01;")
        InfoBar.info(
            title="Info",
            content="Program execution stopped",
            orient=Qt.Horizontal,
            isClosable=True,
            position=InfoBarPosition.TOP_RIGHT,
            duration=2000,
            parent=self
        )

    def _execute_program(self) -> None:
        try:
            for i, target in enumerate(self.compiled_program):
                if not self.program_running:
                    break
                self.program_status_label.setText(
                    f"Step {i+1}/{len(self.compiled_program)}"
                )
                for j in range(min(len(target), self.worker.num_joints)):
                    self.worker.update_target_angle(j, target[j])
                start_time = time.time()
                while self.program_running and (time.time() - start_time) < 5.0:
                    complete = all(
                        abs(self.worker.joint_angles[j] - target[j]) < 1.0
                        for j in range(min(len(target), self.worker.num_joints))
                    )
                    if complete:
                        break
                    time.sleep(0.1)
                time.sleep(self.program_step_delay)
            if self.program_running:
                self.program_status_label.setText("Completed")
                self.program_status_label.setStyleSheet("color: #107C10;")
                InfoBar.success(
                    title="Success",
                    content="Program execution completed",
                    orient=Qt.Horizontal,
                    isClosable=True,
                    position=InfoBarPosition.TOP_RIGHT,
                    duration=2000,
                    parent=self
                )
        except Exception as e:
            self.program_status_label.setText(f"Error: {e}")
            self.program_status_label.setStyleSheet("color: #A4262C;")
            InfoBar.error(
                title="Execution Error",
                content=str(e),
                orient=Qt.Horizontal,
                isClosable=True,
                position=InfoBarPosition.TOP_RIGHT,
                duration=5000,
                parent=self
            )
        self.compile_button.setEnabled(True)
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.program_running = False

    def closeEvent(self, event) -> None:
        if self.program_running:
            self.stop_program()
        if self.worker:
            self.worker.stop()
            # Reset all speed gauges to zero when closing
            for gauge in self.gauges:
                gauge.reset_speed()
            if self.worker_thread and self.worker_thread.is_alive():
                self.worker_thread.join(timeout=1)
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    setTheme(Theme.AUTO)
    sender = RoKiSimSender()
    gui = JointControlGUI(sender)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()