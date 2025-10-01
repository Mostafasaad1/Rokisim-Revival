import logging
import math
import os
import sys
import threading
import time
from typing import List, Optional, Tuple, Dict

from PySide6.QtCore import QObject, QPoint, QRect, Qt, Signal, QTimer, QEasingCurve, QMutex, QMutexLocker        
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
from .plugin_manager import PluginManager
from PySide6.QtWidgets import QListWidget, QListWidgetItem

# ==================== CONSTANTS AND CONFIGURATION ====================
class Config:
    """Configuration constants to avoid magic numbers"""
    JOYSTICK_SIZE = 180
    GAUGE_SIZE = 140
    UPDATE_INTERVAL_MS = 16  # ~60 FPS
    WORKER_UPDATE_INTERVAL = 0.05
    MIN_UPDATE_INTERVAL = 0.1
    SEND_THRESHOLD = 0.1
    ANIMATION_DURATION_MS = 250
    PROGRAM_STEP_DELAY = 0.5
    DEAD_ZONE = 0.1
    MAX_JOYSTICK_DISTANCE = 70


class ColorCache:
    """Cache for theme-aware colors to avoid repeated calculations"""
    _dark_colors = None
    _light_colors = None
    
    @classmethod
    def get_colors(cls, dark_theme: bool) -> Dict[str, QColor]:
        if dark_theme:
            if cls._dark_colors is None:
                cls._dark_colors = {
                    'bg': QColor(45, 45, 45),
                    'border': QColor(80, 80, 80),
                    'handle': QColor(0, 120, 212),
                    'handle_border': QColor(0, 100, 180),
                    'text': QColor(220, 220, 220),
                    'track': QColor(60, 60, 60),
                    'highlight': QColor(0, 120, 212),
                    'progress_start': QColor(0, 150, 136),
                    'progress_end': QColor(0, 120, 212),
                    'card_bg': QColor(30, 30, 30)
                }
            return cls._dark_colors
        else:
            if cls._light_colors is None:
                cls._light_colors = {
                    'bg': QColor(240, 240, 240),
                    'border': QColor(180, 180, 180),
                    'handle': QColor(0, 120, 212),
                    'handle_border': QColor(0, 100, 180),
                    'text': QColor(40, 40, 40),
                    'track': QColor(220, 220, 220),
                    'highlight': QColor(0, 120, 212),
                    'progress_start': QColor(0, 150, 136),
                    'progress_end': QColor(0, 120, 212),
                    'card_bg': QColor(250, 250, 250)
                }
            return cls._light_colors


class FocusAwareDoubleSpinBox(DoubleSpinBox):
    """Optimized spin box with focus signals"""
    
    focused_in = Signal()
    focused_out = Signal()
    
    def focusInEvent(self, event):
        self.focused_in.emit()
        super().focusInEvent(event)
    
    def focusOutEvent(self, event):
        self.focused_out.emit()
        super().focusOutEvent(event)


class JoystickWidget(QWidget):
    """Highly optimized virtual joystick with cached painting"""
    
    joystick_moved = Signal(float, float)
    joystick_released = Signal()
        
    def __init__(self, parent=None, label=""):
        super().__init__(parent)
        self.setFixedSize(Config.JOYSTICK_SIZE, Config.JOYSTICK_SIZE)
        self.setMinimumSize(150, 150)
        # State variables
        self.grab_center = False
        self.moving_offset = QPoint(0, 0)
        self._x = 0.0
        self._y = 0.0
        self.label = label
        # Cached values for painting
        self._cached_theme = None
        self._cached_colors = None
        self._center_point = QPoint(Config.JOYSTICK_SIZE // 2, Config.JOYSTICK_SIZE // 2)
        self._max_distance = Config.MAX_JOYSTICK_DISTANCE
        # Pre-calculated geometry
        self._outer_radius = self._max_distance + 10
        self._handle_radius = 22
        self._cross_extent = self._max_distance + 5
        self.setMouseTracking(True)
        # self.setAttribute(Qt.WA_OpaquePaintEvent)  # Optimize painting
        self.setAttribute(Qt.WA_TranslucentBackground)  
        self.setAutoFillBackground(False)         
        
    def _update_color_cache(self):
        """Update cached colors based on current theme"""
        dark_theme = isDarkTheme()
        if self._cached_theme != dark_theme:
            self._cached_theme = dark_theme
            self._cached_colors = ColorCache.get_colors(dark_theme)
            self.update()  # Repaint when theme changes

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Update color cache
        self._update_color_cache()
        colors = self._cached_colors
        
        # Draw background circle
        painter.setPen(QPen(colors['border'], 2))
        painter.setBrush(colors['bg'])
        # painter.drawEllipse(self._center_point, self._outer_radius, self._outer_radius)
        safe_radius = min(self.width(), self.height()) // 2 - 20
        painter.drawEllipse(self._center_point, safe_radius, safe_radius)
        # Draw crosshair lines
        painter.setPen(QPen(colors['border'], 1))
        center = self._center_point
        painter.drawLine(center.x() - self._cross_extent, center.y(), 
                        center.x() + self._cross_extent, center.y())
        painter.drawLine(center.x(), center.y() - self._cross_extent, 
                        center.x(), center.y() + self._cross_extent)
        
        # Draw handle
        handle_pos = center + self.moving_offset
        painter.setPen(QPen(colors['handle_border'], 2))
        painter.setBrush(colors['handle'])
        painter.drawEllipse(handle_pos, self._handle_radius, self._handle_radius)
        
        # Draw center dot
        painter.setPen(Qt.NoPen)
        painter.setBrush(colors['border'])
        painter.drawEllipse(handle_pos, 4, 4)
        
        # Draw label
        if self.label:
            painter.setPen(colors['text'])
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
        center = self._center_point
        delta = point - center
        
        # Limit to circular boundary using integer math for speed
        dx, dy = delta.x(), delta.y()
        distance_squared = dx * dx + dy * dy
        max_distance_squared = self._max_distance * self._max_distance
        
        if distance_squared > max_distance_squared:
            # Normalize and scale to boundary
            distance = math.sqrt(distance_squared)
            scale = self._max_distance / distance
            dx = int(dx * scale)
            dy = int(dy * scale)
            delta = QPoint(dx, dy)
        
        self.moving_offset = delta
        self._x = delta.x() / self._max_distance
        self._y = -delta.y() / self._max_distance  # Invert Y for natural movement
        
        self.update()
        self.joystick_moved.emit(self._x, self._y)
    
    def get_position(self):
        return self._x, self._y


class GamepadManager(QObject):
    """Highly optimized gamepad manager with efficient polling"""
    
    gamepad_connected = Signal(str)
    gamepad_disconnected = Signal()
    axis_moved = Signal(int, float)
    button_pressed = Signal(int)
    button_released = Signal(int)
    
    def __init__(self):
        super().__init__()
        self.joysticks = []
        self.running = False
        self.thread = None
        self.gamepad_available = GAMEPAD_AVAILABLE
        self._dead_zone = Config.DEAD_ZONE
        
        # State tracking for change detection
        self._last_axis_values = {}
        self._last_button_states = {}
        
        if self.gamepad_available:
            try:
                pygame.init()
                pygame.joystick.init()
                self.check_gamepads()
            except Exception as e:
                logging.error(f"Gamepad initialization failed: {e}")
                self.gamepad_available = False
    
    def check_gamepads(self):
        """Check for connected gamepads with minimal overhead"""
        if not self.gamepad_available:
            return
            
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            if not self.joysticks:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                self.joysticks.append(joystick)
                self.gamepad_connected.emit(joystick.get_name())
        elif self.joysticks:
            self.joysticks.clear()
            self._last_axis_values.clear()
            self._last_button_states.clear()
            self.gamepad_disconnected.emit()
    
    def start_polling(self):
        """Start optimized gamepad polling"""
        if not self.gamepad_available:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._poll_gamepad, daemon=True)
        self.thread.start()
    
    def stop_polling(self):
        """Stop polling and cleanup resources"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.gamepad_available:
            pygame.quit()
    
    def _poll_gamepad(self):
        """Optimized gamepad polling with change detection"""
        poll_interval = 1.0 / 60.0  # 60 FPS
        
        while self.running:
            if self.gamepad_available and self.joysticks:
                try:
                    pygame.event.pump()
                    joystick = self.joysticks[0]
                    
                    # Read axes with dead zone and change detection
                    num_axes = min(4, joystick.get_numaxes())
                    for i in range(num_axes):
                        value = joystick.get_axis(i)
                        if abs(value) > self._dead_zone:
                            last_value = self._last_axis_values.get(i, 0)
                            if abs(value - last_value) > 0.01:  # Significant change threshold
                                self.axis_moved.emit(i, value)
                                self._last_axis_values[i] = value
                        elif i in self._last_axis_values:
                            # Axis returned to center, emit zero and clear
                            self.axis_moved.emit(i, 0.0)
                            del self._last_axis_values[i]
                    
                    # Read buttons with state change detection
                    num_buttons = min(8, joystick.get_numbuttons())
                    for i in range(num_buttons):
                        current_state = joystick.get_button(i)
                        last_state = self._last_button_states.get(i, False)
                        
                        if current_state != last_state:
                            if current_state:
                                self.button_pressed.emit(i)
                            else:
                                self.button_released.emit(i)
                            self._last_button_states[i] = current_state
                            
                except Exception as e:
                    logging.error(f"Gamepad polling error: {e}")
                    self.check_gamepads()  # Recheck gamepad connection
            
            time.sleep(poll_interval)


class SpeedGauge(QWidget):
    """Highly optimized speed gauge with precomputed values"""
    
    def __init__(self, max_speed: float = 50.0, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.max_speed = max_speed
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.joint_name = "Joint"
        
        # Animation
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animate_speed)
        self.animation_timer.setInterval(Config.UPDATE_INTERVAL_MS)
        self.animation_steps = 0
        self.total_steps = 0
        
        # Cached values for painting
        self._cached_theme = None
        self._cached_colors = None
        self._precomputed_geometry = None
        
        self.setFixedSize(Config.GAUGE_SIZE, Config.GAUGE_SIZE)
        # self.setAttribute(Qt.WA_OpaquePaintEvent)  # Optimize painting
        self.setAutoFillBackground(False)

    def set_speed(self, speed: float) -> None:
        self.target_speed = min(abs(speed), self.max_speed)
        if not self.animation_timer.isActive():
            self.animation_steps = 0
            self.total_steps = int(Config.ANIMATION_DURATION_MS / Config.UPDATE_INTERVAL_MS)
            self.animation_timer.start()

    def reset_speed(self) -> None:
        self.set_speed(0.0)

    def set_joint_name(self, name: str) -> None:
        self.joint_name = name
        self.update()

    def _animate_speed(self):
        """Optimized animation with early termination"""
        if self.animation_steps < self.total_steps:
            t = self.animation_steps / self.total_steps
            eased_t = 1 - (1 - t) ** 3  # Ease-out cubic
            new_speed = self.current_speed + (self.target_speed - self.current_speed) * eased_t
            
            # Only update if change is significant
            if abs(new_speed - self.current_speed) > 0.01:
                self.current_speed = new_speed
                self.animation_steps += 1
                self.update()
            else:
                self.animation_steps += 1
        else:
            self.current_speed = self.target_speed
            self.animation_timer.stop()
            self.update()

    def _update_color_cache(self):
        """Update cached colors based on current theme"""
        dark_theme = isDarkTheme()
        if self._cached_theme != dark_theme:
            self._cached_theme = dark_theme
            self._cached_colors = ColorCache.get_colors(dark_theme)
            self._precomputed_geometry = None  # Force geometry recomputation

    def _precompute_geometry(self):
        """Precompute all geometry for painting"""
        if self._precomputed_geometry is not None:
            return
            
        size = min(self.width(), self.height()) - 20
        center = QPoint(self.width() // 2, self.height() // 2)
        radius = size // 2
        
        # Precompute tick positions
        tick_angles = []
        tick_positions = []
        for i in range(0, 181, 45):
            angle_rad = math.radians(i - 90)
            tick_angles.append(angle_rad)
            
            inner_x = center.x() + int((radius - 12) * math.cos(angle_rad))
            inner_y = center.y() + int((radius - 12) * math.sin(angle_rad))
            outer_x = center.x() + int((radius - 5) * math.cos(angle_rad))
            outer_y = center.y() + int((radius - 5) * math.sin(angle_rad))
            tick_positions.append((inner_x, inner_y, outer_x, outer_y))
        
        self._precomputed_geometry = {
            'center': center,
            'radius': radius,
            'size': size,
            'tick_angles': tick_angles,
            'tick_positions': tick_positions
        }

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Update color cache
        self._update_color_cache()
        colors = self._cached_colors
        
        # Precompute geometry if needed
        if self._precomputed_geometry is None:
            self._precompute_geometry()
        
        geom = self._precomputed_geometry
        center = geom['center']
        radius = geom['radius']
        
        # Draw background circle
        painter.setPen(Qt.NoPen)
        painter.setBrush(colors['bg'])
        painter.drawEllipse(center, radius, radius)
        
        # Draw track
        painter.setPen(QPen(colors['track'], 4))
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(
            center.x() - radius, center.y() - radius,
            radius * 2, radius * 2,
            -90 * 16, 180 * 16
        )
        
        # Draw progress arc
        if self.max_speed > 0 and self.current_speed > 0:
            progress_angle = int((self.current_speed / self.max_speed) * 180)
            if progress_angle > 0:  # Only draw if there's progress
                gradient = QLinearGradient(
                    center.x() - radius, center.y(),
                    center.x() + radius, center.y()
                )
                gradient.setColorAt(0, colors['progress_start'])
                gradient.setColorAt(1, colors['progress_end'])
                
                painter.setPen(QPen(gradient, 6))
                painter.drawArc(
                    center.x() - radius, center.y() - radius,
                    radius * 2, radius * 2,
                    -90 * 16, progress_angle * 16
                )
        
        # Draw center circle
        painter.setPen(Qt.NoPen)
        painter.setBrush(colors['bg'])
        painter.drawEllipse(center, radius - 15, radius - 15)
        
        # Draw speed value
        painter.setPen(colors['highlight'])
        painter.setFont(QFont("Segoe UI", 16, QFont.DemiBold))
        speed_text = f"{self.current_speed:.1f}"
        speed_rect = QRect(center.x() - 40, center.y() - 15, 80, 35)
        painter.drawText(speed_rect, Qt.AlignCenter, speed_text)
        
        # Draw unit label
        painter.setPen(colors['text'])
        painter.setFont(QFont("Segoe UI", 9))
        unit_rect = QRect(center.x() - 40, center.y() + 18, 80, 20)
        painter.drawText(unit_rect, Qt.AlignCenter, "°/s")
        
        # Draw joint name
        # painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        # name_rect = QRect(center.x() - 50, center.y() - radius + 5, 100, 25)
        # painter.drawText(name_rect, Qt.AlignCenter, self.joint_name)
        
        # Draw tick marks using precomputed positions
        painter.setPen(QPen(colors['text'], 1))
        for i, (inner_x, inner_y, outer_x, outer_y) in enumerate(geom['tick_positions']):
            painter.drawLine(inner_x, inner_y, outer_x, outer_y)
            
            if i % 2 == 0:  # Only label major ticks (0, 90, 180)
                value = int((i * 45 / 180.0) * self.max_speed)
                label_x = center.x() + int((radius - 25) * math.cos(geom['tick_angles'][i]))
                label_y = center.y() + int((radius - 25) * math.sin(geom['tick_angles'][i]))
                painter.setFont(QFont("Segoe UI", 8))
                label_rect = QRect(label_x - 10, label_y - 8, 20, 16)
                painter.drawText(label_rect, Qt.AlignCenter, str(value))


class Worker(QObject):
    """Highly optimized worker with efficient update logic"""
    
    update_signal = Signal(list)
    fk_update_signal = Signal(tuple, tuple)
    speed_update_signal = Signal(list)

    def __init__(self, sender: RoKiSimSender, num_joints: int, max_speeds: List[float]):
        super().__init__()
        self.sender = sender
        self.num_joints = num_joints
        self.max_speeds = max_speeds
        self.running = True
        
        # Thread-safe data access
        self._mutex = QMutex()
        self.joint_angles: List[float] = [0.0] * num_joints
        self.target_angles: List[float] = [0.0] * num_joints
        self.last_sent_angles: List[float] = [0.0] * num_joints
        self.previous_angles: List[float] = [0.0] * num_joints
        
        # Configuration
        self.global_speed_factor = 1.0
        self.last_update_time = time.time()
        self.last_fk_calculation = 0
        self.last_speed_calculation = 0
        self.fk_calculation_interval = 0.2  # Reduce FK calculation frequency
        self.speed_calculation_interval = Config.WORKER_UPDATE_INTERVAL

    def update_target_angle(self, index: int, value: float) -> None:
        """Thread-safe target angle update"""
        self._mutex.lock()
        try:
            self.target_angles[index] = value
        finally:
            self._mutex.unlock()

    def set_global_speed_factor(self, factor: float) -> None:
        """Set global speed factor"""
        self.global_speed_factor = max(0.1, min(2.0, factor))  # Clamp to reasonable range

    def _interpolate_angle(self, current: float, target: float, max_speed: float, dt: float) -> float:
        """Optimized angle interpolation"""
        delta = target - current
        max_step = max_speed * dt * self.global_speed_factor
        
        if abs(delta) <= max_step:
            return target
        
        return current + math.copysign(max_step, delta)

    def run(self) -> None:
        """Main worker loop with optimized timing"""
        last_loop_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_loop_time
            last_loop_time = current_time
            
            self._mutex.lock()
            try:
                # Store previous angles for speed calculation
                self.previous_angles = self.joint_angles.copy()
                
                # Interpolate angles with change detection
                has_significant_change = False
                for i in range(self.num_joints):
                    new_angle = self._interpolate_angle(
                        self.joint_angles[i],
                        self.target_angles[i],
                        self.max_speeds[i],
                        dt,
                    )
                    
                    # Only update if change is significant
                    if abs(new_angle - self.joint_angles[i]) > 0.001:
                        self.joint_angles[i] = new_angle
                        if abs(new_angle - self.last_sent_angles[i]) > Config.SEND_THRESHOLD:
                            has_significant_change = True
                
                # Send angles if significant change and enough time passed
                if has_significant_change and current_time - self.last_update_time >= Config.MIN_UPDATE_INTERVAL:
                    try:
                        self.sender.send_angles(self.joint_angles)
                        self.last_sent_angles = self.joint_angles.copy()
                        self.last_update_time = current_time
                        self.update_signal.emit(self.joint_angles.copy())
                    except Exception as e:
                        logging.error(f"Error sending angles: {e}")
                
                # Calculate FK less frequently to save CPU
                if current_time - self.last_fk_calculation >= self.fk_calculation_interval:
                    pos, rpy = self.sender.calculate_fk(self.joint_angles)
                    if pos and rpy:
                        self.fk_update_signal.emit(pos, rpy)
                        self.last_fk_calculation = current_time
                
                # Calculate speeds at fixed interval
                if current_time - self.last_speed_calculation >= self.speed_calculation_interval:
                    speeds = []
                    for i in range(self.num_joints):
                        angle_change = abs(self.joint_angles[i] - self.previous_angles[i])
                        speed = angle_change / dt if dt > 0 else 0.0
                        speeds.append(speed)
                    self.speed_update_signal.emit(speeds)
                    self.last_speed_calculation = current_time
                    
            finally:
                self._mutex.unlock()
            
            # Adaptive sleep to maintain timing
            elapsed = time.time() - current_time
            sleep_time = max(0, Config.WORKER_UPDATE_INTERVAL - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self) -> None:
        """Stop worker and cleanup"""
        self.running = False
        self.speed_update_signal.emit([0.0] * self.num_joints)


class JointControlGUI(FluentWindow):
    """Optimized main GUI class with efficient resource management"""
    
    def __init__(self, sender: RoKiSimSender):
        super().__init__()
        self.sender = sender
        self.worker: Optional[Worker] = None
        self.worker_thread: Optional[threading.Thread] = None
        
        # Control state
        self.user_interaction_active = False
        self.joint_edit_flags = []
        self.slider_update_timers = []
        self.reset_in_progress = False
        
        # Joystick control
        self.joystick_mode = "joint"
        self.joystick_target_joint = 0
        self.joystick_sensitivity = 2.0
        self.cartesian_sensitivity = {"xy": 10.0, "z": 5.0, "rot": 2.0}
        
        # Program execution
        self.compiled_program: Optional[List[List[float]]] = None
        self.program_running = False
        self.program_thread: Optional[threading.Thread] = None
        self.compiler = InstructionSetCompiler()
        
        # Gamepad
        self.gamepad_manager = GamepadManager() if GAMEPAD_AVAILABLE else None
        
        # UI components
        self.sliders: List[Slider] = []
        self.spin_boxes: List[FocusAwareDoubleSpinBox] = []
        self.gauges: List[SpeedGauge] = []
        self.fk_labels: List[StrongBodyLabel] = []
        self.ik_entries: List[LineEdit] = []
        
        self._setup_ui()
        self._connect_signals()

    def _setup_ui(self):
        """Setup the main UI components"""
        self.setWindowTitle("RoKiSim Robot Controller")
        self.resize(1200, 800)
        
        # Create tabs
        self.joint_tab = self._create_joint_tab()
        self.joint_tab.setObjectName("joint_tab")             # <-- ADDED

        self.program_tab = self._create_program_tab()
        self.program_tab.setObjectName("program_tab")         # <-- ADDED

        self.joystick_tab = self._create_joystick_tab()
        self.joystick_tab.setObjectName("joystick_tab")       # <-- ADDED
            
        self.plugins_tab = self._create_plugins_tab()
        self.plugins_tab.setObjectName("plugins_tab")
           
        # Add to navigation
        self.addSubInterface(self.joint_tab, FluentIcon.ROBOT, 'Joint Control')
        self.addSubInterface(self.joystick_tab, FluentIcon.GAME, 'Joystick Control')
        self.addSubInterface(self.program_tab, FluentIcon.CODE, 'Program Editor')
        self.addSubInterface(self.plugins_tab, FluentIcon.APPLICATION, 'Plugins')
        
        # Add plugins
        self._add_plugin_tabs()
        
        # Set theme
        setTheme(Theme.AUTO)
        setThemeColor('#0078D4')

    def _connect_signals(self):
        """Connect signals for gamepad and other events"""
        if self.gamepad_manager:
            self.gamepad_manager.gamepad_connected.connect(self._on_gamepad_connected)
            self.gamepad_manager.gamepad_disconnected.connect(self._on_gamepad_disconnected)
            self.gamepad_manager.axis_moved.connect(self._on_gamepad_axis_moved)
            self.gamepad_manager.start_polling()

    def _add_plugin_tabs(self):
        """Dynamically add tabs for plugins that provide UI panels."""
        ui_plugins = self.plugin_manager.get_ui_plugins()
        for name, plugin in ui_plugins.items():
            try:
                panel = plugin.create_ui_panel(self)
                if panel:
                    # Set valid objectName
                    safe_name = "".join(c if c.isalnum() else "_" for c in name)
                    panel.setObjectName(f"plugin_tab_{safe_name}")

                    # 🔑 Use plugin's custom icon
                    icon = plugin.get_icon()

                    self.addSubInterface(
                        panel,
                        icon,
                        plugin.get_ui_panel_title()
                    )
                    setattr(self, f"plugin_tab_{safe_name}", panel)
            except Exception as e:
                logging.error(f"Failed to create UI for plugin {name}: {e}")
                InfoBar.error("Plugin Error", f"Failed to load UI for {name}", parent=self)
    
    def get_joint_angles(self) -> List[float]:
        """Safely return a copy of current joint angles."""
        if not self.worker:
            return []
        from PySide6.QtCore import QMutexLocker
        with QMutexLocker(self.worker._mutex):
            return self.worker.joint_angles.copy()

    def set_joint_target(self, index: int, angle: float) -> bool:
        """Safely set target angle for one joint."""
        if not self.worker or not (0 <= index < self.worker.num_joints):
            return False
        self.worker.update_target_angle(index, angle)
        if not self.joint_edit_flags[index]:
            self.sliders[index].setValue(int(angle * 10))
            self.spin_boxes[index].setValue(angle)
        return True

    def set_all_joint_targets(self, angles: List[float]) -> bool:
        """Safely set target angles for all joints."""
        if not self.worker or len(angles) != self.worker.num_joints:
            return False
        for i, a in enumerate(angles):
            self.worker.update_target_angle(i, a)
            if not self.joint_edit_flags[i]:
                self.sliders[i].setValue(int(a * 10))
                self.spin_boxes[i].setValue(a)
        return True

    def _create_joint_tab(self) -> QWidget:
        """Create optimized joint control tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
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
        
        # Content area
        content_widget = QWidget()
        content_layout = QHBoxLayout(content_widget)
        content_layout.setSpacing(25)
        
        # Left panel - Joint controls
        left_panel = self._create_joint_control_panel()
        content_layout.addWidget(left_panel)
        
        # Right panel - FK/IK controls
        right_panel = self._create_fk_ik_panel()
        content_layout.addWidget(right_panel)
        
        layout.addWidget(content_widget)
        return tab

    def _create_joint_control_panel(self) -> CardWidget:
        """Create joint control panel"""
        panel = CardWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)
        
        # Title
        joint_title = SubtitleLabel("Joint Controls")
        layout.addWidget(joint_title)
        
        # Joint controls container
        self.joint_controls_card = CardWidget()
        self.joint_controls_layout = QVBoxLayout(self.joint_controls_card)
        self.joint_controls_layout.setContentsMargins(15, 15, 15, 15)
        self.joint_controls_layout.setSpacing(12)
        
        # Scroll area
        scroll = SmoothScrollArea()
        scroll.setWidget(self.joint_controls_card)
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("SmoothScrollArea { border: none; }")
        layout.addWidget(scroll)
        
        return panel

    def _create_fk_ik_panel(self) -> QWidget:
        """Create FK/IK control panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setSpacing(25)
        
        # Speed control
        speed_card = self._create_speed_control_card()
        layout.addWidget(speed_card)
        
        # Forward Kinematics
        fk_card = self._create_fk_card()
        layout.addWidget(fk_card)
        
        # Inverse Kinematics
        ik_card = self._create_ik_card()
        layout.addWidget(ik_card)
        
        return panel

    def _create_speed_control_card(self) -> CardWidget:
        """Create speed control card"""
        card = CardWidget()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 15, 20, 15)
        layout.setSpacing(12)
        
        title = SubtitleLabel("Global Speed Control")
        layout.addWidget(title)
        
        speed_layout = QHBoxLayout()
        speed_layout.setSpacing(15)
        speed_layout.addWidget(BodyLabel("Speed Factor:"))
        
        self.speed_factor_slider = Slider(Qt.Horizontal)
        self.speed_factor_slider.setFixedHeight(24)
        self.speed_factor_slider.setRange(1, 20)
        self.speed_factor_slider.setValue(10)
        self.speed_factor_slider.valueChanged.connect(self.update_global_speed)
        speed_layout.addWidget(self.speed_factor_slider)
        
        self.speed_factor_label = BodyLabel("1.0x")
        self.speed_factor_label.setFixedWidth(40)
        speed_layout.addWidget(self.speed_factor_label)
        
        layout.addLayout(speed_layout)
        return card

    def _create_fk_card(self) -> CardWidget:
        """Create forward kinematics card"""
        card = CardWidget()
        layout = QGridLayout(card)
        layout.setContentsMargins(20, 15, 20, 15)
        layout.setSpacing(15)
        
        title = SubtitleLabel("Forward Kinematics")
        layout.addWidget(title, 0, 0, 1, 2)
        
        # Create FK labels
        self.fk_labels = [StrongBodyLabel("0.00") for _ in range(6)]
        fk_labels_text = ["X:", "Y:", "Z:", "Roll:", "Pitch:", "Yaw:"]
        
        for i in range(6):
            row = (i // 2) + 1
            col = (i % 2) * 2
            layout.addWidget(BodyLabel(fk_labels_text[i]), row, col)
            layout.addWidget(self.fk_labels[i], row, col + 1)
            
        return card

    def _create_ik_card(self) -> CardWidget:
        """Create inverse kinematics card"""
        card = CardWidget()
        layout = QGridLayout(card)
        layout.setContentsMargins(20, 15, 20, 15)
        layout.setSpacing(15)
        
        title = SubtitleLabel("Inverse Kinematics")
        layout.addWidget(title, 0, 0, 1, 2)
        
        # Create IK entries
        self.ik_entries = [LineEdit() for _ in range(6)]
        ik_labels_text = ["X:", "Y:", "Z:", "Roll:", "Pitch:", "Yaw:"]
        
        for i, entry in enumerate(self.ik_entries):
            row = (i // 2) + 1
            col = (i % 2) * 2
            layout.addWidget(BodyLabel(ik_labels_text[i]), row, col)
            entry.setValidator(QDoubleValidator())
            entry.setFixedHeight(35)
            layout.addWidget(entry, row, col + 1)
        
        # IK button
        self.ik_button = PrimaryPushButton(FluentIcon.MOVE, "Move to Pose (IK)")
        self.ik_button.setFixedHeight(35)
        self.ik_button.clicked.connect(self.move_to_ik_pose)
        layout.addWidget(self.ik_button, 4, 0, 1, 2)
        
        # Status label
        self.ik_status_label = BodyLabel("")
        self.ik_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.ik_status_label, 5, 0, 1, 2)
        
        return card

    def _create_joystick_tab(self) -> QWidget:
        """Create optimized joystick control tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(25)
        
        # Title
        title_label = TitleLabel("Dual Joystick Control")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # Control mode selection
        mode_card = self._create_mode_selection_card()
        layout.addWidget(mode_card)
        
        # Main content
        main_content = CardWidget()
        main_layout = QVBoxLayout(main_content)
        main_layout.setContentsMargins(10, 25, 30, 25)
        main_layout.setSpacing(30)
        
        # Mode stack
        self.mode_stack = QWidget()
        self.mode_stack_layout = QVBoxLayout(self.mode_stack)
        self.mode_stack_layout.setContentsMargins(0, 0, 0, 0)
        self.mode_stack_layout.setSpacing(20)
        
        # Create mode widgets
        self.joint_mode_widget = self._create_joint_mode_widget()
        self.cartesian_mode_widget = self._create_cartesian_mode_widget()
        
        self.mode_stack_layout.addWidget(self.joint_mode_widget)
        self.mode_stack_layout.addWidget(self.cartesian_mode_widget)
        
        main_layout.addWidget(self.mode_stack)
        layout.addWidget(main_content)
        
        # Show initial mode
        self._update_mode_visibility()
        
        return tab

    def _create_mode_selection_card(self) -> CardWidget:
        """Create mode selection card"""
        card = CardWidget()
        layout = QHBoxLayout(card)
        layout.setContentsMargins(20, 15, 20, 15)
        layout.setSpacing(20)
        
        layout.addWidget(BodyLabel("Control Mode:"))
        
        self.mode_combo = ComboBox()
        self.mode_combo.setFixedWidth(180)
        self.mode_combo.addItems(["Individual Joint Control", "Cartesian (X-Y-Z) Control"])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        layout.addWidget(self.mode_combo)
        
        layout.addStretch()
        return card

    def _create_joint_mode_widget(self) -> QWidget:
        """Create joint control mode widget"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setSpacing(40)
        
        # Virtual joystick
        self.joint_joystick = JoystickWidget(label="Joint Control")
        self.joint_joystick.joystick_moved.connect(self._on_joint_joystick_moved)
        self.joint_joystick.joystick_released.connect(self._on_joystick_released)
        layout.addWidget(self.joint_joystick, alignment=Qt.AlignCenter)
        
        # Control panel
        control_panel = self._create_joint_control_panel_joystick()
        layout.addWidget(control_panel)
        
        return widget

    def _create_joint_control_panel_joystick(self) -> CardWidget:
        """Create joint control panel for joystick tab"""
        panel = CardWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)
        
        # Joint selection
        joint_layout = QHBoxLayout()
        joint_layout.addWidget(BodyLabel("Control Joint:"))
        self.joystick_joint_combo = ComboBox()
        self.joystick_joint_combo.setFixedWidth(120)
        joint_layout.addWidget(self.joystick_joint_combo)
        layout.addLayout(joint_layout)
        
        # Sensitivity
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
        layout.addLayout(sensitivity_layout)
        
        # Enable toggle
        self.joystick_toggle = SwitchButton("Enable Joystick")
        self.joystick_toggle.checkedChanged.connect(self._on_joystick_toggled)
        layout.addWidget(self.joystick_toggle)
        
        # Gamepad status
        if GAMEPAD_AVAILABLE:
            self.gamepad_status = BodyLabel("🎮 Gamepad: Checking...")
            layout.addWidget(self.gamepad_status)
        else:
            self.gamepad_status = BodyLabel("🎮 Gamepad support not available")
            self.gamepad_status.setStyleSheet("color: #888888;")
            layout.addWidget(self.gamepad_status)
        
        return panel

    def _create_cartesian_mode_widget(self) -> QWidget:
        """Create cartesian control mode widget"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setSpacing(30)
        
        # Left joystick for X-Y
        self.xy_joystick = JoystickWidget(label="X-Y Plane")
        self.xy_joystick.joystick_moved.connect(self._on_xy_joystick_moved)
        self.xy_joystick.joystick_released.connect(self._on_joystick_released)
        layout.addWidget(self.xy_joystick, alignment=Qt.AlignCenter)
        
        # Right joystick for Z-Rotation
        self.z_rot_joystick = JoystickWidget(label="Z & Rotation")
        self.z_rot_joystick.joystick_moved.connect(self._on_z_rot_joystick_moved)
        self.z_rot_joystick.joystick_released.connect(self._on_joystick_released)
        layout.addWidget(self.z_rot_joystick, alignment=Qt.AlignCenter)
        
        # Control panel
        control_panel = self._create_cartesian_control_panel()
        layout.addWidget(control_panel)
        
        return widget

    def _create_cartesian_control_panel(self) -> CardWidget:
        """Create cartesian control panel"""
        panel = CardWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)
        
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
        layout.addLayout(xy_sensitivity_layout)
        
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
        layout.addLayout(z_sensitivity_layout)
        
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
        layout.addLayout(rot_sensitivity_layout)
        
        # Enable toggle
        self.cartesian_toggle = SwitchButton("Enable Cartesian Control")
        self.cartesian_toggle.checkedChanged.connect(self._on_cartesian_toggled)
        layout.addWidget(self.cartesian_toggle)
        
        # Instructions
        instructions = BodyLabel(
            "• Left joystick: Move in X-Y plane\n"
            "• Right joystick: Control Z height and rotation\n"
            "• Use gamepad left/right sticks for same control"
        )
        instructions.setStyleSheet("color: #666666; font-size: 12px;")
        layout.addWidget(instructions)
        
        return panel

    def _create_program_tab(self) -> QWidget:
        """Create program editor tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(25)
        
        # Title
        title_label = TitleLabel("Robot Program Editor")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # Editor card
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
        
        # Buttons
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
        
        # Output card
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
        
        return tab

    def _create_plugins_tab(self) -> QWidget:
        """Create Plugins tab for plugin management"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(25)

        title_label = TitleLabel("Plugin Manager")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # Plugin list
        self.plugin_list = QListWidget()
        self.plugin_list.setStyleSheet("font-size: 13px;")
        layout.addWidget(self.plugin_list)

        # Controls
        ctrl_layout = QHBoxLayout()
        self.refresh_btn = PrimaryPushButton("Refresh Plugins")
        self.refresh_btn.clicked.connect(self._refresh_plugins)
        ctrl_layout.addWidget(self.refresh_btn)

        self.execute_btn = PrimaryPushButton("Execute Selected")
        self.execute_btn.clicked.connect(self._execute_selected_plugin)
        ctrl_layout.addWidget(self.execute_btn)

        self.args_input = LineEdit()
        self.args_input.setPlaceholderText("Arguments (comma-separated, e.g. Alice, Bob)")
        ctrl_layout.addWidget(self.args_input)

        layout.addLayout(ctrl_layout)

        # Output
        self.plugin_output = PlainTextEdit()
        self.plugin_output.setReadOnly(True)
        layout.addWidget(self.plugin_output)

        self._refresh_plugins()
        return tab

    # ==================== EVENT HANDLERS ====================

    def _on_gamepad_connected(self, name: str):
        """Handle gamepad connection"""
        if hasattr(self, 'gamepad_status'):
            self.gamepad_status.setText(f"🎮 Gamepad: {name}")
        
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
        """Handle gamepad disconnection"""
        if hasattr(self, 'gamepad_status'):
            self.gamepad_status.setText("🎮 Gamepad: Disconnected")
        
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
        """Handle gamepad axis movement"""
        if not self.worker or self.joystick_mode == "disabled":
            return
            
        if self.joystick_mode == "joint" and self.joystick_toggle.isChecked():
            # Joint control mode
            target_angle = self.worker.joint_angles[self.joystick_target_joint] + (value * self.joystick_sensitivity)
            self.worker.update_target_angle(self.joystick_target_joint, target_angle)
            
            if not self.joint_edit_flags[self.joystick_target_joint]:
                self.sliders[self.joystick_target_joint].setValue(int(target_angle * 10))
                self.spin_boxes[self.joystick_target_joint].setValue(target_angle)
                    
        elif self.joystick_mode == "cartesian" and self.cartesian_toggle.isChecked():
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
            self._move_to_cartesian_pose(target_pose)

    def _on_mode_changed(self, index: int):
        """Handle control mode change"""
        self._update_mode_visibility()
        
        # Update internal mode
        self.joystick_mode = "joint" if index == 0 else "cartesian"
        
        # Show notification
        mode_name = "Joint" if index == 0 else "Cartesian"
        InfoBar.info(
            title="Mode Changed",
            content=f"Switched to {mode_name} control mode",
            orient=Qt.Horizontal,
            isClosable=True,
            position=InfoBarPosition.TOP_RIGHT,
            duration=2000,
            parent=self
        )

    def _update_mode_visibility(self):
        """Update mode widget visibility"""
        mode_index = self.mode_combo.currentIndex()
        self.joint_mode_widget.setVisible(mode_index == 0)
        self.cartesian_mode_widget.setVisible(mode_index == 1)

    def _on_joint_joystick_moved(self, x: float, y: float):
        """Handle joint joystick movement"""
        if self.joystick_mode == "joint" and self.joystick_toggle.isChecked() and self.worker:
            target_angle = self.worker.joint_angles[self.joystick_target_joint] + (x * self.joystick_sensitivity)
            self.worker.update_target_angle(self.joystick_target_joint, target_angle)
            
            if not self.joint_edit_flags[self.joystick_target_joint]:
                self.sliders[self.joystick_target_joint].setValue(int(target_angle * 10))
                self.spin_boxes[self.joystick_target_joint].setValue(target_angle)

    def _on_xy_joystick_moved(self, x: float, y: float):
        """Handle XY joystick movement"""
        if self.joystick_mode == "cartesian" and self.cartesian_toggle.isChecked() and self.worker:
            current_fk = self._get_current_fk()
            if not current_fk:
                return
                
            # Update X and Y based on joystick
            new_x = current_fk[0] + (x * self.cartesian_sensitivity["xy"])
            new_y = current_fk[1] - (y * self.cartesian_sensitivity["xy"])  # Invert Y
            
            # Keep other values the same
            target_pose = (new_x, new_y, current_fk[2], current_fk[3], current_fk[4], current_fk[5])
            self._move_to_cartesian_pose(target_pose)

    def _on_z_rot_joystick_moved(self, x: float, y: float):
        """Handle Z-Rotation joystick movement"""
        if self.joystick_mode == "cartesian" and self.cartesian_toggle.isChecked() and self.worker:
            current_fk = self._get_current_fk()
            if not current_fk:
                return
                
            # Update Z and Yaw based on joystick
            new_z = current_fk[2] + (x * self.cartesian_sensitivity["z"])
            new_yaw = current_fk[5] - (y * self.cartesian_sensitivity["rot"])  # Invert Y
            
            # Keep other values the same
            target_pose = (current_fk[0], current_fk[1], new_z, current_fk[3], current_fk[4], new_yaw)
            self._move_to_cartesian_pose(target_pose)

    def _on_joystick_released(self):
        """Handle joystick release - no action needed"""
        pass

    def _on_sensitivity_changed(self, value: int):
        """Handle joint sensitivity change"""
        self.joystick_sensitivity = float(value)
        self.sensitivity_label.setText(f"{value}.0°")

    def _on_xy_sensitivity_changed(self, value: int):
        """Handle XY sensitivity change"""
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
        """Handle joystick enable/disable"""
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
        """Handle cartesian control enable/disable"""
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

    # ==================== UTILITY METHODS ====================

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

    def _refresh_plugins(self):
        """Refresh plugin list from PluginManager"""
        self.plugin_manager = PluginManager()
        self.plugin_list.clear()
        for name, meta in self.plugin_manager.list_plugins().items():
            item = QListWidgetItem(f"{name} v{meta['version']} – {meta['description']} (by {meta['author']})")
            item.setData(Qt.UserRole, name)
            self.plugin_list.addItem(item)

    def _execute_selected_plugin(self):
        current = self.plugin_list.currentItem()
        if not current:
            InfoBar.warning("No Plugin", "Please select a plugin.", parent=self)
            return
        plugin_name = current.data(Qt.UserRole)
        args_str = self.args_input.text().strip()
        args = [arg.strip() for arg in args_str.split(",")] if args_str else []

        try:
            # Pass the GUI as 'gui' so plugins can call gui.get_joint_angles(), etc.
            result = self.plugin_manager.execute_plugin(plugin_name, *args, gui=self)
            self.plugin_output.setPlainText(f"✅ Result:\n{result}")
            InfoBar.success("Success", f"Plugin '{plugin_name}' executed.", parent=self)
        except Exception as e:
            error_msg = f"❌ Error:\n{str(e)}"
            self.plugin_output.setPlainText(error_msg)
            InfoBar.error("Plugin Error", str(e), parent=self)
            
    # ==================== JOINT CONTROL METHODS ====================

    def load_robot_xml(self) -> None:
        """Load robot definition from XML file"""
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

    def _create_joint_controls(self, num_joints: int) -> None:
        """Create joint control widgets"""
        # Clear existing controls
        for i in reversed(range(self.joint_controls_layout.count())):
            widget = self.joint_controls_layout.itemAt(i).widget()
            if widget:
                widget.deleteLater()
        
        # Initialize lists
        self.sliders.clear()
        self.spin_boxes.clear()
        self.gauges.clear()
        self.joint_edit_flags = [False] * num_joints
        self.slider_update_timers = [None] * num_joints
        
        # Get joint limits
        min_angles, max_angles = self.sender.robot_definition.get_joint_limits()
        
        # Create controls for each joint
        for i in range(num_joints):
            joint_row = self._create_joint_control_row(i, min_angles[i], max_angles[i])
            self.joint_controls_layout.addWidget(joint_row)
        
        self.joint_controls_layout.addStretch()

    def _create_joint_control_row(self, index: int, min_angle: float, max_angle: float) -> CardWidget:
        """Create a single joint control row"""
        joint_row = CardWidget()
        layout = QHBoxLayout(joint_row)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(20)
        
        # Joint info
        info_widget = self._create_joint_info_widget(index, min_angle, max_angle)
        layout.addWidget(info_widget)
        
        # Slider
        slider = Slider(Qt.Horizontal)
        slider.setRange(int(min_angle * 10), int(max_angle * 10))
        slider.setValue(0)
        slider.setFixedHeight(26)
        slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        
        slider.sliderPressed.connect(lambda checked=False, idx=index: self.on_slider_pressed(idx))
        slider.sliderReleased.connect(lambda checked=False, idx=index: self.on_slider_released(idx))
        slider.valueChanged.connect(lambda v, idx=index: self.on_slider_value_changed(idx, v))
        
        self.sliders.append(slider)
        layout.addWidget(slider, 1)
        
        # Spin box
        spin_box = FocusAwareDoubleSpinBox()
        spin_box.setRange(min_angle, max_angle)
        spin_box.setValue(0.0)
        spin_box.setFixedWidth(100)
        spin_box.setFixedHeight(35)
        spin_box.setSuffix("°")
        spin_box.setStyleSheet("font-size: 13px;")
        
        spin_box.focused_in.connect(lambda idx=index: self.on_spin_box_focus_in(idx))
        spin_box.focused_out.connect(lambda idx=index: self.on_spin_box_focus_out(idx))
        spin_box.valueChanged.connect(lambda v, idx=index: self.update_target_from_spin(idx, v))
        
        self.spin_boxes.append(spin_box)
        layout.addWidget(spin_box)
        
        # Reset button
        reset_button = PushButton(FluentIcon.RETURN, "Reset")
        reset_button.setFixedWidth(80)
        reset_button.setFixedHeight(35)
        reset_button.clicked.connect(lambda _, idx=index: self.reset_joint(idx))
        layout.addWidget(reset_button)
        
        # Speed gauge
        gauge = SpeedGauge()
        gauge.set_joint_name(f"J{index+1}")
        self.gauges.append(gauge)
        layout.addWidget(gauge)
        
        return joint_row

    def _create_joint_info_widget(self, index: int, min_angle: float, max_angle: float) -> QWidget:
        """Create joint information widget"""
        info_widget = QWidget()
        info_layout = QVBoxLayout(info_widget)
        info_layout.setSpacing(4)
        
        joint_label = StrongBodyLabel(f"Joint {index+1}")
        info_layout.addWidget(joint_label)
        
        range_label = BodyLabel(f"[{min_angle:.1f}°, {max_angle:.1f}°]")
        range_label.setStyleSheet("font-size: 11px; color: #888888;")
        info_layout.addWidget(range_label)
        
        info_widget.setFixedWidth(120)
        return info_widget

    def on_slider_pressed(self, index: int) -> None:
        """Handle slider press"""
        self.joint_edit_flags[index] = True
        self.user_interaction_active = True

    def on_slider_released(self, index: int) -> None:
        """Handle slider release"""
        if self.slider_update_timers[index] is not None:
            self.slider_update_timers[index].stop()
        
        value = self.sliders[index].value() / 10.0
        if self.worker:
            self.worker.update_target_angle(index, value)
            self.spin_boxes[index].setValue(value)
        
        self.joint_edit_flags[index] = False
        self.user_interaction_active = any(self.joint_edit_flags)

    def on_slider_value_changed(self, index: int, value: int) -> None:
        """Handle slider value change with debouncing"""
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
        """Debounced slider update"""
        if self.worker and self.joint_edit_flags[index]:
            self.worker.update_target_angle(index, value / 10.0)

    def on_spin_box_focus_in(self, index: int) -> None:
        """Handle spin box focus in"""
        self.joint_edit_flags[index] = True
        self.user_interaction_active = True

    def on_spin_box_focus_out(self, index: int) -> None:
        """Handle spin box focus out"""
        self.joint_edit_flags[index] = False
        self.user_interaction_active = any(self.joint_edit_flags)

    def update_target_from_spin(self, index: int, value: float) -> None:
        """Update target angle from spin box"""
        if self.joint_edit_flags[index] and self.worker:
            self.worker.update_target_angle(index, value)
            self.sliders[index].setValue(int(value * 10))

    def reset_joint(self, index: int) -> None:
        """Reset joint to zero position"""
        if self.worker:
            self.reset_in_progress = True
            self.worker.update_target_angle(index, 0.0)
            self.spin_boxes[index].setValue(0.0)
            self.sliders[index].setValue(0)
            QTimer.singleShot(100, lambda: setattr(self, 'reset_in_progress', False))

    def _start_worker(self, num_joints: int) -> None:
        """Start worker thread"""
        max_speeds = [50.0] * num_joints
        self.worker = Worker(self.sender, num_joints, max_speeds)
        self.worker.update_signal.connect(self.update_gui_from_worker)
        self.worker.fk_update_signal.connect(self.update_fk_display)
        self.worker.speed_update_signal.connect(self.update_speed_gauges)
        self.worker_thread = threading.Thread(target=self.worker.run, daemon=True)
        self.worker_thread.start()

    def update_gui_from_worker(self, angles: List[float]) -> None:
        """Update GUI from worker thread"""
        if self.reset_in_progress or self.user_interaction_active:
            return
            
        for i, angle in enumerate(angles):
            if not self.joint_edit_flags[i]:
                # Block signals to prevent feedback loops
                self.sliders[i].blockSignals(True)
                self.spin_boxes[i].blockSignals(True)
                
                self.sliders[i].setValue(int(angle * 10))
                self.spin_boxes[i].setValue(angle)
                
                self.sliders[i].blockSignals(False)
                self.spin_boxes[i].blockSignals(False)

    def update_speed_gauges(self, speeds: List[float]) -> None:
        """Update speed gauges"""
        for i, speed in enumerate(speeds):
            self.gauges[i].set_speed(speed)

    def update_fk_display(self, pos: Tuple[float, ...], rpy: Tuple[float, ...]) -> None:
        """Update forward kinematics display"""
        values = list(pos) + list(rpy)
        for i, label in enumerate(self.fk_labels):
            label.setText(f"{values[i]:.2f}")

    def update_global_speed(self, value: int) -> None:
        """Update global speed factor"""
        factor = value / 10.0
        self.speed_factor_label.setText(f"{factor:.1f}x")
        if self.worker:
            self.worker.set_global_speed_factor(factor)

    def move_to_ik_pose(self) -> None:
        """Move to IK pose"""
        try:
            # Validate input
            if any(not entry.text().strip() for entry in self.ik_entries):
                raise ValueError("Please fill all fields")
            
            target_values = [float(entry.text()) for entry in self.ik_entries]
            target_pose = tuple(target_values)
            
            # Calculate IK solution
            ik_solution = self.sender.calculate_ik(
                target_pose, self.worker.joint_angles if self.worker else None
            )
            
            if ik_solution:
                # Store current edit flags
                original_flags = self.joint_edit_flags.copy()
                self.joint_edit_flags = [False] * len(self.joint_edit_flags)
                
                # Update all joints
                for i, angle in enumerate(ik_solution):
                    self.update_target_from_spin(i, angle)
                
                # Restore edit flags
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

    # ==================== PROGRAM EXECUTION METHODS ====================

    def compile_program(self) -> None:
        """Compile robot program"""
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
            external_handlers = self.plugin_manager.get_instruction_extensions()
            # Initialize compiler
            self.compiler = InstructionSetCompiler(
                self.compiler.default_interpolation_steps,
                external_handlers=external_handlers
            )
            
            # Set current pose
            if self.worker:
                self.compiler.current_pose = self.worker.joint_angles[:]
            
            # Compile program
            self.compiled_program = self.compiler.compile(program_text)
            
            # Update UI
            self.program_status_label.setText(
                f"Compiled: {len(self.compiled_program)} positions"
            )
            self.program_status_label.setStyleSheet("color: #107C10;")
            self.run_button.setEnabled(True)
            
            # Show compiled positions
            self.output_display.setPlainText(
                "Compiled positions:\n" + "\n".join(
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
        """Run compiled program"""
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
        """Stop program execution"""
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
        """Execute compiled program in background thread"""
        try:
            for i, target in enumerate(self.compiled_program):
                if not self.program_running:
                    break
                
                # Update status
                self.program_status_label.setText(
                    f"Step {i+1}/{len(self.compiled_program)}"
                )
                
                # Set target angles
                for j in range(min(len(target), self.worker.num_joints)):
                    self.worker.update_target_angle(j, target[j])
                
                # Wait for movement to complete
                start_time = time.time()
                while self.program_running and (time.time() - start_time) < 5.0:
                    # Check if movement is complete
                    complete = all(
                        abs(self.worker.joint_angles[j] - target[j]) < 1.0
                        for j in range(min(len(target), self.worker.num_joints))
                    )
                    if complete:
                        break
                    time.sleep(0.1)
                
                # Step delay
                time.sleep(Config.PROGRAM_STEP_DELAY)
            
            # Program completed successfully
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
        
        # Reset UI state
        self.compile_button.setEnabled(True)
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.program_running = False

    def closeEvent(self, event) -> None:
        """Handle application close event"""
        # Stop program execution
        if self.program_running:
            self.stop_program()
        
        # Stop worker thread
        if self.worker:
            self.worker.stop()
            for gauge in self.gauges:
                gauge.reset_speed()
        
        # Stop gamepad polling
        if self.gamepad_manager:
            self.gamepad_manager.stop_polling()
        
        # Clear lists to help garbage collection
        self.sliders.clear()
        self.spin_boxes.clear()
        self.gauges.clear()
        self.fk_labels.clear()
        self.ik_entries.clear()
        
        super().closeEvent(event)


def main() -> None:
    """Main application entry point"""
    # Enable high DPI scaling
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    app = QApplication(sys.argv)
    setTheme(Theme.AUTO)
    
    sender = RoKiSimSender()
    gui = JointControlGUI(sender)
    gui.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()