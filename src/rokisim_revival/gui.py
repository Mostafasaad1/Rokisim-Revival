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
    QApplication, QDoubleSpinBox, QFileDialog, QFrame, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QMessageBox, QPlainTextEdit, QPushButton, QScrollArea,
    QSlider, QTabWidget, QTextEdit, QVBoxLayout, QWidget, QGridLayout,
    QGraphicsDropShadowEffect, QSizePolicy
)

# Import gamepad support if available
GAMEPAD_AVAILABLE = False
try:
    import pygame
    GAMEPAD_AVAILABLE = True
except ImportError:
    print("Pygame not available - gamepad support disabled")

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


class JoystickWidget(QWidget):
    """Enhanced virtual joystick widget for robot control"""
    
    joystick_moved = Signal(float, float)  # x, y normalized values (-1.0 to 1.0)
    joystick_released = Signal()
    
    def __init__(self, parent=None, label=""):
        super().__init__(parent)
        self.setFixedSize(180, 180)
        self.setMinimumSize(150, 150)
        self.grab_center = False
        self.__max_distance = 70
        self.moving_offset = QPoint(0, 0)
        self._x = 0.0
        self._y = 0.0
        self.label = label
        
        # Enable mouse tracking for smooth movement
        self.setMouseTracking(True)
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Get center point
        center = QPoint(self.width() // 2, self.height() // 2)
        
        # Draw outer circle (background)
        if isDarkTheme():
            bg_color = QColor(45, 45, 45)
            border_color = QColor(80, 80, 80)
            handle_color = QColor(0, 120, 212)
            handle_border = QColor(0, 100, 180)
            text_color = QColor(220, 220, 220)
        else:
            bg_color = QColor(240, 240, 240)
            border_color = QColor(180, 180, 180)
            handle_color = QColor(0, 120, 212)
            handle_border = QColor(0, 100, 180)
            text_color = QColor(40, 40, 40)
        
        # Draw background
        painter.setPen(QPen(border_color, 2))
        painter.setBrush(bg_color)
        painter.drawEllipse(center, self.__max_distance + 10, self.__max_distance + 10)
        
        # Draw crosshair lines
        painter.setPen(QPen(border_color, 1))
        painter.drawLine(
            center.x() - self.__max_distance - 5, center.y(),
            center.x() + self.__max_distance + 5, center.y()
        )
        painter.drawLine(
            center.x(), center.y() - self.__max_distance - 5,
            center.x(), center.y() + self.__max_distance + 5
        )
        
        # Draw handle
        handle_pos = center + self.moving_offset
        painter.setPen(QPen(handle_border, 2))
        painter.setBrush(handle_color)
        painter.drawEllipse(handle_pos, 22, 22)
        
        # Draw center dot
        painter.setPen(Qt.NoPen)
        painter.setBrush(border_color)
        painter.drawEllipse(handle_pos, 4, 4)
        
        # Draw label
        if self.label:
            painter.setPen(text_color)
            painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
            label_rect = QRect(0, 10, self.width(), 25)
            painter.drawText(label_rect, Qt.AlignCenter, self.label)
        
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.grab_center = True
            self._update_offset(event.pos())
            event.accept()
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.grab_center = False
            self.moving_offset = QPoint(0, 0)
            self._x = 0.0
            self._y = 0.0
            self.update()
            self.joystick_released.emit()
            event.accept()
    
    def mouseMoveEvent(self, event):
        if self.grab_center:
            self._update_offset(event.pos())
            event.accept()
    
    def _update_offset(self, point):
        center = QPoint(self.width() // 2, self.height() // 2)
        delta = point - center
        
        # Limit to circular boundary
        distance = math.sqrt(delta.x()**2 + delta.y()**2)
        if distance > self.__max_distance:
            delta = delta * (self.__max_distance / distance)
        
        self.moving_offset = delta
        self._x = delta.x() / self.__max_distance
        self._y = -delta.y() / self.__max_distance  # Invert Y for natural movement
        
        self.update()
        self.joystick_moved.emit(self._x, self._y)
    
    def get_position(self):
        """Get current joystick position as normalized values (-1.0 to 1.0)"""
        return self._x, self._y


class GamepadManager(QObject):
    """Manages physical gamepad input"""
    
    gamepad_connected = Signal(str)
    gamepad_disconnected = Signal()
    axis_moved = Signal(int, float)  # axis, value
    button_pressed = Signal(int)
    button_released = Signal(int)
    
    def __init__(self):
        super().__init__()
        self.joysticks = []
        self.running = False
        self.thread = None
        self.gamepad_available = GAMEPAD_AVAILABLE
        
        if self.gamepad_available:
            try:
                pygame.init()
                pygame.joystick.init()
                self.check_gamepads()
            except:
                self.gamepad_available = False
    
    def check_gamepads(self):
        """Check for connected gamepads"""
        if not self.gamepad_available:
            return
            
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            if len(self.joysticks) == 0:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                self.joysticks.append(joystick)
                self.gamepad_connected.emit(joystick.get_name())
        else:
            if self.joysticks:
                self.joysticks = []
                self.gamepad_disconnected.emit()
    
    def start_polling(self):
        """Start gamepad polling thread"""
        if not self.gamepad_available:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._poll_gamepad, daemon=True)
        self.thread.start()
    
    def stop_polling(self):
        """Stop gamepad polling"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
    
    def _poll_gamepad(self):
        """Poll gamepad events in background thread"""
        while self.running:
            if self.gamepad_available and self.joysticks:
                try:
                    pygame.event.pump()
                    joystick = self.joysticks[0]
                    
                    # Read all axes (0=left stick X, 1=left stick Y, 2=right stick X, 3=right stick Y)
                    for i in range(min(4, joystick.get_numaxes())):
                        value = joystick.get_axis(i)
                        if abs(value) > 0.1:  # Dead zone
                            self.axis_moved.emit(i, value)
                    
                    # Read buttons
                    for i in range(min(4, joystick.get_numbuttons())):
                        if joystick.get_button(i):
                            self.button_pressed.emit(i)
                        else:
                            self.button_released.emit(i)
                            
                except Exception as e:
                    logging.error(f"Gamepad error: {e}")
            
            time.sleep(0.016)  # ~60 FPS


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
            bg_color = QColor(30, 30, 30)
            track_color = QColor(60, 60, 60)
            text_color = QColor(220, 220, 220)
            highlight_color = QColor(0, 120, 212)
            progress_start = QColor(0, 150, 136)
            progress_end = QColor(0, 120, 212)
        else:
            bg_color = QColor(250, 250, 250)
            track_color = QColor(220, 220, 220)
            text_color = QColor(40, 40, 40)
            highlight_color = QColor(0, 120, 212)
            progress_start = QColor(0, 150, 136)
            progress_end = QColor(0, 120, 212)
        
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
        
        # Draw speed value
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
        for i in range(0, 181, 45):
            angle = math.radians(i - 90)
            inner_x = center.x() + (radius - 12) * math.cos(angle)
            inner_y = center.y() + (radius - 12) * math.sin(angle)
            outer_x = center.x() + (radius - 5) * math.cos(angle)
            outer_y = center.y() + (radius - 5) * math.sin(angle)
            painter.drawLine(QPoint(int(inner_x), int(inner_y)), QPoint(int(outer_x), int(outer_y)))
            
            if i % 90 == 0:
                value = int((i / 180.0) * self.max_speed)
                label_x = center.x() + (radius - 25) * math.cos(angle)
                label_y = center.y() + (radius - 25) * math.sin(angle)
                painter.setFont(QFont("Segoe UI", 8))
                label_rect = QRect(int(label_x) - 10, int(label_y) - 8, 20, 16)
                painter.drawText(label_rect, Qt.AlignCenter, str(value))


class Worker(QObject):
    update_signal = Signal(list)
    fk_update_signal = Signal(tuple, tuple)
    speed_update_signal = Signal(list)

    def __init__(self, sender: RoKiSimSender, num_joints: int, max_speeds: List[float]):
        super().__init__()
        self.sender = sender
        self.num_joints = num_joints
        self.max_speeds = max_speeds
        self.running = True
        self.joint_angles: List[float] = [0.0] * num_joints
        self.target_angles: List[float] = [0.0] * num_joints
        self.last_sent_angles: List[float] = [0.0] * num_joints
        self.previous_angles: List[float] = [0.0] * num_joints
        self.UPDATE_INTERVAL = 0.05
        self.SEND_THRESHOLD = 0.1
        self.global_speed_factor = 1.0
        self.last_update_time = time.time()
        self.MIN_UPDATE_INTERVAL = 0.1

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
            
            if current_time - self.last_update_time >= self.MIN_UPDATE_INTERVAL:
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
                
                speeds = []
                for i in range(self.num_joints):
                    angle_change = abs(self.joint_angles[i] - self.previous_angles[i])
                    speed = angle_change / self.UPDATE_INTERVAL if self.UPDATE_INTERVAL > 0 else 0.0
                    speeds.append(speed)
                self.speed_update_signal.emit(speeds)
            
            time.sleep(self.UPDATE_INTERVAL)

    def stop(self) -> None:
        self.running = False
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
        self.program_step_delay = 0.5
        
        # Control state tracking
        self.user_interaction_active = False
        self.joint_edit_flags = []
        self.slider_update_timers = []
        self.reset_in_progress = False
        
        # Joystick control modes
        self.joystick_mode = "joint"  # "joint", "cartesian", or "disabled"
        self.joystick_target_joint = 0
        self.joystick_sensitivity = 2.0
        self.cartesian_sensitivity = {"xy": 10.0, "z": 5.0, "rot": 2.0}
        
        # Initialize gamepad manager
        self.gamepad_manager = GamepadManager() if GAMEPAD_AVAILABLE else None
        
        # Set window properties
        self.setWindowTitle("RoKiSim Robot Controller")
        self.resize(1200, 800)
        
        # Create interfaces
        self.joint_tab = QWidget()
        self.joint_tab.setObjectName("jointTab")
        self.program_tab = QWidget()
        self.program_tab.setObjectName("programTab")
        self.joystick_tab = QWidget()
        self.joystick_tab.setObjectName("joystickTab")
        
        self._setup_joint_tab()
        self._setup_program_tab()
        self._setup_joystick_tab()
        
        # Add interfaces to navigation
        self.addSubInterface(self.joint_tab, FluentIcon.ROBOT, 'Joint Control')
        self.addSubInterface(self.joystick_tab, FluentIcon.GAME, 'Joystick Control')
        self.addSubInterface(self.program_tab, FluentIcon.CODE, 'Program Editor')
        
        # Set theme and colors
        setTheme(Theme.AUTO)
        setThemeColor('#0078D4')
        
        # Initialize gamepad if available
        if self.gamepad_manager:
            self.gamepad_manager.gamepad_connected.connect(self._on_gamepad_connected)
            self.gamepad_manager.gamepad_disconnected.connect(self._on_gamepad_disconnected)
            self.gamepad_manager.axis_moved.connect(self._on_gamepad_axis_moved)
            self.gamepad_manager.start_polling()

    def _on_gamepad_connected(self, name: str):
        InfoBar.success(
            title="Gamepad Connected",
            content=f"Connected to {name}",
            orient=Qt.Horizontal,
            isClosable=True,
            position=InfoBarPosition.TOP_RIGHT,
            duration=3000,
            parent=self
        )

    def _on_gamepad_disconnected(self):
        InfoBar.warning(
            title="Gamepad Disconnected",
            content="Gamepad disconnected",
            orient=Qt.Horizontal,
            isClosable=True,
            position=InfoBarPosition.TOP_RIGHT,
            duration=2000,
            parent=self
        )

    def _on_gamepad_axis_moved(self, axis: int, value: float):
        """Handle gamepad axis movement for dual joystick control"""
        if self.joystick_mode == "disabled" or not self.worker:
            return
            
        if self.joystick_mode == "joint":
            # Joint control mode - use left stick X-axis
            if axis == 0:
                target_angle = self.worker.joint_angles[self.joystick_target_joint] + (value * self.joystick_sensitivity)
                self.worker.update_target_angle(self.joystick_target_joint, target_angle)
                if not self.joint_edit_flags[self.joystick_target_joint]:
                    self.sliders[self.joystick_target_joint].setValue(int(target_angle * 10))
                    self.spin_boxes[self.joystick_target_joint].setValue(target_angle)
                    
        elif self.joystick_mode == "cartesian":
            # Cartesian control mode
            current_fk = self._get_current_fk()
            if not current_fk:
                return
                
            x, y, z, roll, pitch, yaw = current_fk
            
            if axis == 0:  # Left stick X - X movement
                x += value * self.cartesian_sensitivity["xy"]
            elif axis == 1:  # Left stick Y - Y movement (inverted)
                y -= value * self.cartesian_sensitivity["xy"]
            elif axis == 2:  # Right stick X - Z movement
                z += value * self.cartesian_sensitivity["z"]
            elif axis == 3:  # Right stick Y - Rotation (inverted)
                yaw -= value * self.cartesian_sensitivity["rot"]
            
            # Calculate IK solution
            target_pose = (x, y, z, roll, pitch, yaw)
            ik_solution = self.sender.calculate_ik(target_pose, self.worker.joint_angles)
            
            if ik_solution:
                # Update all joints
                for i, angle in enumerate(ik_solution):
                    self.worker.update_target_angle(i, angle)
                    if not self.joint_edit_flags[i]:
                        self.sliders[i].setValue(int(angle * 10))
                        self.spin_boxes[i].setValue(angle)

    def _setup_joint_tab(self) -> None:
        layout = QVBoxLayout(self.joint_tab)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(25)
        
        title_label = TitleLabel("Robot Joint Control")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        self.load_button = PrimaryPushButton(FluentIcon.FOLDER, "Load Robot Definition")
        self.load_button.setFixedHeight(45)
        self.load_button.clicked.connect(self.load_robot_xml)
        layout.addWidget(self.load_button)
        
        content_widget = QWidget()
        content_layout = QHBoxLayout(content_widget)
        content_layout.setSpacing(25)
        
        # Left panel - Joint controls
        left_panel = CardWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(20, 20, 20, 20)
        left_layout.setSpacing(15)
        
        joint_title = SubtitleLabel("Joint Controls")
        left_layout.addWidget(joint_title)
        
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

    def _setup_joystick_tab(self) -> None:
        """Setup the enhanced dual joystick control tab"""
        layout = QVBoxLayout(self.joystick_tab)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(25)
        
        title_label = TitleLabel("Dual Joystick Control")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # Control mode selection
        mode_card = CardWidget()
        mode_layout = QHBoxLayout(mode_card)
        mode_layout.setContentsMargins(20, 15, 20, 15)
        mode_layout.setSpacing(20)
        
        mode_layout.addWidget(BodyLabel("Control Mode:"))
        
        self.mode_combo = ComboBox()
        self.mode_combo.setFixedWidth(180)
        self.mode_combo.addItems(["Individual Joint Control", "Cartesian (X-Y-Z) Control"])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self.mode_combo)
        
        mode_layout.addStretch()
        layout.addWidget(mode_card)
        
        # Main content area
        main_content = CardWidget()
        main_layout = QVBoxLayout(main_content)
        main_layout.setContentsMargins(10, 25, 30, 25)
        main_layout.setSpacing(30)
        
        # Stack for different modes
        self.mode_stack = QWidget()
        self.mode_stack_layout = QVBoxLayout(self.mode_stack)
        self.mode_stack_layout.setContentsMargins(0, 0, 0, 0)
        self.mode_stack_layout.setSpacing(20)
        
        # Joint Control Mode
        self.joint_mode_widget = self._create_joint_mode_widget()
        self.mode_stack_layout.addWidget(self.joint_mode_widget)
        
        # Cartesian Control Mode
        self.cartesian_mode_widget = self._create_cartesian_mode_widget()
        self.mode_stack_layout.addWidget(self.cartesian_mode_widget)
        
        main_layout.addWidget(self.mode_stack)
        layout.addWidget(main_content)
        
        # Show appropriate mode initially
        self._update_mode_visibility()

    def _create_joint_mode_widget(self):
        """Create widget for individual joint control mode"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setSpacing(40)
        
        # Virtual joystick for joint control
        self.joint_joystick = JoystickWidget(label="Joint Control")
        self.joint_joystick.joystick_moved.connect(self._on_joint_joystick_moved)
        self.joint_joystick.joystick_released.connect(self._on_joystick_released)
        layout.addWidget(self.joint_joystick, alignment=Qt.AlignCenter)
        
        # Control panel
        control_panel = CardWidget()
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(20, 20, 20, 20)
        control_layout.setSpacing(20)
        
        # Joint selection
        joint_selection_layout = QHBoxLayout()
        joint_selection_layout.addWidget(BodyLabel("Control Joint:"))
        self.joystick_joint_combo = ComboBox()
        self.joystick_joint_combo.setFixedWidth(120)
        joint_selection_layout.addWidget(self.joystick_joint_combo)
        control_layout.addLayout(joint_selection_layout)
        
        # Sensitivity control
        sensitivity_layout = QHBoxLayout()
        sensitivity_layout.addWidget(BodyLabel("Sensitivity:"))
        self.joystick_sensitivity_slider = Slider(Qt.Horizontal)
        self.joystick_sensitivity_slider.setFixedWidth(150)
        self.joystick_sensitivity_slider.setRange(1, 10)
        self.joystick_sensitivity_slider.setValue(2)
        self.joystick_sensitivity_slider.valueChanged.connect(self._on_sensitivity_changed)
        sensitivity_layout.addWidget(self.joystick_sensitivity_slider)
        self.sensitivity_label = BodyLabel("2.0°")
        self.sensitivity_label.setFixedWidth(50)
        sensitivity_layout.addWidget(self.sensitivity_label)
        control_layout.addLayout(sensitivity_layout)
        
        # Enable toggle
        self.joystick_toggle = SwitchButton("Enable Joystick")
        self.joystick_toggle.checkedChanged.connect(self._on_joystick_toggled)
        control_layout.addWidget(self.joystick_toggle)
        
        # Gamepad status
        if GAMEPAD_AVAILABLE:
            self.gamepad_status = BodyLabel("🎮 Gamepad: Checking...")
            control_layout.addWidget(self.gamepad_status)
        else:
            self.gamepad_status = BodyLabel("🎮 Gamepad support not available")
            self.gamepad_status.setStyleSheet("color: #888888;")
            control_layout.addWidget(self.gamepad_status)
        
        layout.addWidget(control_panel)
        return widget

    def _create_cartesian_mode_widget(self):
        """Create widget for cartesian control mode"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setSpacing(30)
        
        # Left joystick for X-Y control
        self.xy_joystick = JoystickWidget(label="X-Y Plane")
        self.xy_joystick.joystick_moved.connect(self._on_xy_joystick_moved)
        self.xy_joystick.joystick_released.connect(self._on_joystick_released)
        layout.addWidget(self.xy_joystick, alignment=Qt.AlignCenter)
        
        # Right joystick for Z and rotation control
        self.z_rot_joystick = JoystickWidget(label="Z & Rotation")
        self.z_rot_joystick.joystick_moved.connect(self._on_z_rot_joystick_moved)
        self.z_rot_joystick.joystick_released.connect(self._on_joystick_released)
        layout.addWidget(self.z_rot_joystick, alignment=Qt.AlignCenter)
        
        # Control panel
        control_panel = CardWidget()
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(20, 20, 20, 20)
        control_layout.setSpacing(20)
        
        # Sensitivity controls
        xy_sensitivity_layout = QHBoxLayout()
        xy_sensitivity_layout.addWidget(BodyLabel("X-Y Sensitivity:"))
        self.xy_sensitivity_slider = Slider(Qt.Horizontal)
        self.xy_sensitivity_slider.setFixedWidth(120)
        self.xy_sensitivity_slider.setRange(1, 20)
        self.xy_sensitivity_slider.setValue(10)
        self.xy_sensitivity_slider.valueChanged.connect(self._on_xy_sensitivity_changed)
        xy_sensitivity_layout.addWidget(self.xy_sensitivity_slider)
        self.xy_sensitivity_label = BodyLabel("10.0")
        self.xy_sensitivity_label.setFixedWidth(40)
        xy_sensitivity_layout.addWidget(self.xy_sensitivity_label)
        control_layout.addLayout(xy_sensitivity_layout)
        
        z_sensitivity_layout = QHBoxLayout()
        z_sensitivity_layout.addWidget(BodyLabel("Z Sensitivity:"))
        self.z_sensitivity_slider = Slider(Qt.Horizontal)
        self.z_sensitivity_slider.setFixedWidth(120)
        self.z_sensitivity_slider.setRange(1, 20)
        self.z_sensitivity_slider.setValue(5)
        self.z_sensitivity_slider.valueChanged.connect(self._on_z_sensitivity_changed)
        z_sensitivity_layout.addWidget(self.z_sensitivity_slider)
        self.z_sensitivity_label = BodyLabel("5.0")
        self.z_sensitivity_label.setFixedWidth(40)
        z_sensitivity_layout.addWidget(self.z_sensitivity_label)
        control_layout.addLayout(z_sensitivity_layout)
        
        rot_sensitivity_layout = QHBoxLayout()
        rot_sensitivity_layout.addWidget(BodyLabel("Rot Sensitivity:"))
        self.rot_sensitivity_slider = Slider(Qt.Horizontal)
        self.rot_sensitivity_slider.setFixedWidth(120)
        self.rot_sensitivity_slider.setRange(1, 10)
        self.rot_sensitivity_slider.setValue(2)
        self.rot_sensitivity_slider.valueChanged.connect(self._on_rot_sensitivity_changed)
        rot_sensitivity_layout.addWidget(self.rot_sensitivity_slider)
        self.rot_sensitivity_label = BodyLabel("2.0")
        self.rot_sensitivity_label.setFixedWidth(40)
        rot_sensitivity_layout.addWidget(self.rot_sensitivity_label)
        control_layout.addLayout(rot_sensitivity_layout)
        
        # Enable toggle
        self.cartesian_toggle = SwitchButton("Enable Cartesian Control")
        self.cartesian_toggle.checkedChanged.connect(self._on_cartesian_toggled)
        control_layout.addWidget(self.cartesian_toggle)
        
        # Instructions
        instructions = BodyLabel(
            "• Left joystick: Move in X-Y plane\n"
            "• Right joystick: Control Z height and rotation\n"
            "• Use gamepad left/right sticks for same control"
        )
        instructions.setStyleSheet("color: #666666; font-size: 12px;")
        control_layout.addWidget(instructions)
        
        layout.addWidget(control_panel)
        return widget

    def _update_mode_visibility(self):
        """Show/hide appropriate mode widgets based on selection"""
        mode_index = self.mode_combo.currentIndex()
        
        if mode_index == 0:  # Joint control mode
            self.joint_mode_widget.show()
            self.cartesian_mode_widget.hide()
            self.joystick_mode = "joint"
        else:  # Cartesian control mode
            self.joint_mode_widget.hide()
            self.cartesian_mode_widget.show()
            self.joystick_mode = "cartesian"

    def _on_mode_changed(self, index: int):
        """Handle mode selection change"""
        self._update_mode_visibility()
        
        # Update toggle state
        if index == 0:
            enabled = self.joystick_toggle.isChecked()
        else:
            enabled = self.cartesian_toggle.isChecked()
            
        if enabled:
            InfoBar.info(
                title="Mode Changed",
                content="Switched to " + ("Joint" if index == 0 else "Cartesian") + " control mode",
                orient=Qt.Horizontal,
                isClosable=True,
                position=InfoBarPosition.TOP_RIGHT,
                duration=2000,
                parent=self
            )

    def _on_joint_joystick_moved(self, x: float, y: float):
        """Handle joint control joystick movement"""
        if self.joystick_mode == "joint" and self.joystick_toggle.isChecked() and self.worker:
            target_angle = self.worker.joint_angles[self.joystick_target_joint] + (x * self.joystick_sensitivity)
            self.worker.update_target_angle(self.joystick_target_joint, target_angle)
            if not self.joint_edit_flags[self.joystick_target_joint]:
                self.sliders[self.joystick_target_joint].setValue(int(target_angle * 10))
                self.spin_boxes[self.joystick_target_joint].setValue(target_angle)

    def _on_xy_joystick_moved(self, x: float, y: float):
        """Handle X-Y plane joystick movement"""
        if self.joystick_mode == "cartesian" and self.cartesian_toggle.isChecked() and self.worker:
            current_fk = self._get_current_fk()
            if not current_fk:
                return
                
            # Update X and Y based on joystick
            new_x = current_fk[0] + (x * self.cartesian_sensitivity["xy"])
            new_y = current_fk[1] - (y * self.cartesian_sensitivity["xy"])  # Invert Y for natural movement
            
            # Keep other values the same
            target_pose = (new_x, new_y, current_fk[2], current_fk[3], current_fk[4], current_fk[5])
            self._move_to_cartesian_pose(target_pose)

    def _on_z_rot_joystick_moved(self, x: float, y: float):
        """Handle Z and rotation joystick movement"""
        if self.joystick_mode == "cartesian" and self.cartesian_toggle.isChecked() and self.worker:
            current_fk = self._get_current_fk()
            if not current_fk:
                return
                
            # Update Z and Yaw based on joystick
            new_z = current_fk[2] + (x * self.cartesian_sensitivity["z"])
            new_yaw = current_fk[5] - (y * self.cartesian_sensitivity["rot"])  # Invert Y for natural movement
            
            # Keep other values the same
            target_pose = (current_fk[0], current_fk[1], new_z, current_fk[3], current_fk[4], new_yaw)
            self._move_to_cartesian_pose(target_pose)

    def _get_current_fk(self):
        """Get current forward kinematics values"""
        if not self.worker:
            return None
        pos, rpy = self.sender.calculate_fk(self.worker.joint_angles)
        if pos and rpy:
            return list(pos) + list(rpy)
        return None

    def _move_to_cartesian_pose(self, target_pose):
        """Move to cartesian pose using IK"""
        ik_solution = self.sender.calculate_ik(target_pose, self.worker.joint_angles)
        if ik_solution:
            for i, angle in enumerate(ik_solution):
                self.worker.update_target_angle(i, angle)
                if not self.joint_edit_flags[i]:
                    self.sliders[i].setValue(int(angle * 10))
                    self.spin_boxes[i].setValue(angle)
        else:
            pass

            # InfoBar.warning(
            #     title="IK Failed",
            #     content="Cannot reach target position",
            #     orient=Qt.Horizontal,
            #     isClosable=True,
            #     position=InfoBarPosition.TOP_RIGHT,
            #     duration=2000,
            #     parent=self
            # )

    def _on_joystick_released(self):
        """Handle joystick release - no action needed"""
        pass

    def _on_sensitivity_changed(self, value: int):
        """Handle joint sensitivity change"""
        self.joystick_sensitivity = float(value)
        self.sensitivity_label.setText(f"{value}.0°")

    def _on_xy_sensitivity_changed(self, value: int):
        """Handle X-Y sensitivity change"""
        self.cartesian_sensitivity["xy"] = float(value)
        self.xy_sensitivity_label.setText(f"{value}.0")

    def _on_z_sensitivity_changed(self, value: int):
        """Handle Z sensitivity change"""
        self.cartesian_sensitivity["z"] = float(value)
        self.z_sensitivity_label.setText(f"{value}.0")

    def _on_rot_sensitivity_changed(self, value: int):
        """Handle rotation sensitivity change"""
        self.cartesian_sensitivity["rot"] = float(value)
        self.rot_sensitivity_label.setText(f"{value}.0")

    def _on_joystick_toggled(self, checked: bool):
        """Handle joint joystick enable/disable"""
        if self.mode_combo.currentIndex() == 0:  # Only affect when in joint mode
            if checked:
                InfoBar.info(
                    title="Joystick Enabled",
                    content=f"Controlling Joint {self.joystick_target_joint + 1}",
                    orient=Qt.Horizontal,
                    isClosable=True,
                    position=InfoBarPosition.TOP_RIGHT,
                    duration=2000,
                    parent=self
                )
            else:
                InfoBar.info(
                    title="Joystick Disabled",
                    content="Joint control deactivated",
                    orient=Qt.Horizontal,
                    isClosable=True,
                    position=InfoBarPosition.TOP_RIGHT,
                    duration=2000,
                    parent=self
                )

    def _on_cartesian_toggled(self, checked: bool):
        """Handle cartesian joystick enable/disable"""
        if self.mode_combo.currentIndex() == 1:  # Only affect when in cartesian mode
            if checked:
                InfoBar.info(
                    title="Cartesian Control Enabled",
                    content="X-Y-Z control active",
                    orient=Qt.Horizontal,
                    isClosable=True,
                    position=InfoBarPosition.TOP_RIGHT,
                    duration=2000,
                    parent=self
                )
            else:
                InfoBar.info(
                    title="Cartesian Control Disabled",
                    content="Cartesian control deactivated",
                    orient=Qt.Horizontal,
                    isClosable=True,
                    position=InfoBarPosition.TOP_RIGHT,
                    duration=2000,
                    parent=self
                )

    def _setup_program_tab(self) -> None:
        layout = QVBoxLayout(self.program_tab)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(25)
        
        title_label = TitleLabel("Robot Program Editor")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
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
                
                # Update joystick joint combo
                self.joystick_joint_combo.clear()
                for i in range(num_joints):
                    self.joystick_joint_combo.addItem(f"Joint {i+1}")
                self.joystick_joint_combo.currentIndexChanged.connect(self._on_joystick_joint_changed)
                
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

    def _on_joystick_joint_changed(self, index: int):
        """Handle joystick joint selection change"""
        self.joystick_target_joint = index
        if self.joystick_mode == "joint" and self.joystick_toggle.isChecked():
            InfoBar.info(
                title="Joint Changed",
                content=f"Now controlling Joint {index + 1}",
                orient=Qt.Horizontal,
                isClosable=True,
                position=InfoBarPosition.TOP_RIGHT,
                duration=1500,
                parent=self
            )

    def _create_joint_controls(self, num_joints: int) -> None:
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
            
            slider = Slider(Qt.Horizontal)
            slider.setRange(int(min_angles[i] * 10), int(max_angles[i] * 10))
            slider.setValue(0)
            slider.setFixedHeight(26)
            slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            
            slider.sliderPressed.connect(lambda checked=False, idx=i: self.on_slider_pressed(idx))
            slider.sliderReleased.connect(lambda checked=False, idx=i: self.on_slider_released(idx))
            slider.valueChanged.connect(lambda v, idx=i: self.on_slider_value_changed(idx, v))
            
            self.sliders.append(slider)
            joint_row_layout.addWidget(slider, 1)
            
            spin_box = FocusAwareDoubleSpinBox()
            spin_box.setRange(min_angles[i], max_angles[i])
            spin_box.setValue(0.0)
            spin_box.setFixedWidth(100)
            spin_box.setFixedHeight(35)
            spin_box.setSuffix("°")
            spin_box.setStyleSheet("font-size: 13px;")
            
            spin_box.focused_in.connect(lambda idx=i: self.on_spin_box_focus_in(idx))
            spin_box.focused_out.connect(lambda idx=i: self.on_spin_box_focus_out(idx))
            spin_box.valueChanged.connect(lambda v, idx=i: self.update_target_from_spin(idx, v))
            
            self.spin_boxes.append(spin_box)
            joint_row_layout.addWidget(spin_box)
            
            reset_button = PushButton(FluentIcon.RETURN, "Reset")
            reset_button.setFixedWidth(80)
            reset_button.setFixedHeight(35)
            reset_button.clicked.connect(lambda _, idx=i: self.reset_joint(idx))
            self.reset_buttons.append(reset_button)
            joint_row_layout.addWidget(reset_button)
            
            gauge = SpeedGauge()
            gauge.set_joint_name(f"J{i+1}")
            self.gauges.append(gauge)
            joint_row_layout.addWidget(gauge)
            
            self.joint_controls_layout.addWidget(joint_row)
        
        self.joint_controls_layout.addStretch()

    # ... (rest of the methods remain the same as before)
    # on_slider_pressed, on_slider_released, on_slider_value_changed, etc.
    # update_global_speed, on_spin_box_focus_in, on_spin_box_focus_out, etc.
    # update_target_from_spin, reset_joint, _start_worker, etc.
    # update_gui_from_worker, update_speed_gauges, update_fk_display, etc.
    # move_to_ik_pose, compile_program, run_program, stop_program, etc.
    # _execute_program, closeEvent

    def on_slider_pressed(self, index: int) -> None:
        self.joint_edit_flags[index] = True
        self.user_interaction_active = True

    def on_slider_released(self, index: int) -> None:
        if self.slider_update_timers[index] is not None:
            self.slider_update_timers[index].stop()
        
        value = self.sliders[index].value() / 10.0
        if self.worker:
            self.worker.update_target_angle(index, value)
            self.spin_boxes[index].setValue(value)
        
        self.joint_edit_flags[index] = False
        self.user_interaction_active = any(self.joint_edit_flags)

    def on_slider_value_changed(self, index: int, value: int) -> None:
        if not self.joint_edit_flags[index]:
            return
            
        if self.slider_update_timers[index] is not None:
            self.slider_update_timers[index].stop()
        
        timer = QTimer()
        timer.setSingleShot(True)
        timer.timeout.connect(lambda: self.debounced_slider_update(index, value))
        timer.start(50)
        self.slider_update_timers[index] = timer
        
        self.spin_boxes[index].setValue(value / 10.0)

    def debounced_slider_update(self, index: int, value: int) -> None:
        if self.worker and self.joint_edit_flags[index]:
            self.worker.update_target_angle(index, value / 10.0)

    def on_spin_box_focus_in(self, index: int) -> None:
        self.joint_edit_flags[index] = True
        self.user_interaction_active = True

    def on_spin_box_focus_out(self, index: int) -> None:
        self.joint_edit_flags[index] = False
        self.user_interaction_active = any(self.joint_edit_flags)

    def update_target_from_spin(self, index: int, value: float) -> None:
        if self.joint_edit_flags[index] and self.worker:
            self.worker.update_target_angle(index, value)
            self.sliders[index].setValue(int(value * 10))

    def reset_joint(self, index: int) -> None:
        if self.worker:
            self.reset_in_progress = True
            self.worker.update_target_angle(index, 0.0)
            self.spin_boxes[index].setValue(0.0)
            self.sliders[index].setValue(0)
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
        if self.reset_in_progress:
            return
            
        for i, angle in enumerate(angles):
            if not self.joint_edit_flags[i]:
                self.sliders[i].blockSignals(True)
                self.spin_boxes[i].blockSignals(True)
                self.sliders[i].setValue(int(angle * 10))
                self.spin_boxes[i].setValue(angle)
                self.sliders[i].blockSignals(False)
                self.spin_boxes[i].blockSignals(False)

    def update_speed_gauges(self, speeds: List[float]) -> None:
        for i, speed in enumerate(speeds):
            self.gauges[i].set_speed(speed)

    def update_fk_display(self, pos: Tuple[float, ...], rpy: Tuple[float, ...]) -> None:
        values = list(pos) + list(rpy)
        for i, label in enumerate(self.fk_labels):
            label.setText(f"{values[i]:.2f}")

    def move_to_ik_pose(self) -> None:
        try:
            if any(not entry.text().strip() for entry in self.ik_entries):
                raise ValueError("Empty field")

            target_values = [float(entry.text()) for entry in self.ik_entries]
            target_pose = tuple(target_values)
            ik_solution = self.sender.calculate_ik(
                target_pose, self.worker.joint_angles if self.worker else None
            )
            if ik_solution:
                original_flags = self.joint_edit_flags.copy()
                self.joint_edit_flags = [False] * len(self.joint_edit_flags)
                
                for i, angle in enumerate(ik_solution):
                    self.update_target_from_spin(i, angle)
                
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

    def update_global_speed(self, value: int) -> None:
        """Update global speed factor"""
        factor = value / 10.0
        self.speed_factor_label.setText(f"{factor:.1f}x")
        if self.worker:
            self.worker.set_global_speed_factor(factor)


    def closeEvent(self, event) -> None:
        if self.program_running:
            self.stop_program()
        if self.worker:
            self.worker.stop()
            for gauge in self.gauges:
                gauge.reset_speed()
        if self.gamepad_manager:
            self.gamepad_manager.stop_polling()
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    setTheme(Theme.DARK)
    sender = RoKiSimSender()
    gui = JointControlGUI(sender)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()