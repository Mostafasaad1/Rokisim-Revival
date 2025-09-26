# joint_control_gui.py
"""
PySide6 GUI for controlling robot joints with real-time, smooth interpolation.
Uses RoKiSimSender for communication and kinematics.
Includes integrated robot programming language compiler.
"""
import sys
import threading
import time
import logging
import os
import math
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                               QLabel, QSlider, QPushButton, QLineEdit, QGroupBox,
                               QFileDialog, QMessageBox, QScrollArea, QDoubleSpinBox,
                               QFrame, QTabWidget, QPlainTextEdit)
from PySide6.QtCore import Qt, Signal, QObject, QRect, QPoint
from PySide6.QtGui import QDoubleValidator, QPainter, QPen, QColor, QFont

import rokisim_sender  # Assumes rokisim_sender.py is in the same directory

# Import the EnhancedRobotCompiler
try:
    from instructionSet import EnhancedRobotCompiler
except ImportError:
    # Fallback if instructionSet.py is not available
    class EnhancedRobotCompiler:
        def __init__(self, interpolation_steps=10):
            self.interpolation_steps = interpolation_steps
            self.positions = {}
            self.variables = {}
            self.output = []
            
        def compile(self, source_code):
            # Dummy implementation
            return [[0, 0, 0, 0, 0, 0]]
            
        def _preprocess(self, source_code):
            return []
            
        def _first_pass(self, lines):
            pass
            
        def _second_pass(self, lines):
            pass


class SpeedGauge(QWidget):
    """Circular gauge widget to display joint speed like a car meter"""
    
    def __init__(self, max_speed=50.0, parent=None):
        super().__init__(parent)
        self.setMinimumSize(120, 120)
        self.max_speed = max_speed
        self.current_speed = 0.0
        self.joint_name = "Joint"
        
    def set_speed(self, speed):
        self.current_speed = min(speed, self.max_speed)
        self.update()
        
    def set_joint_name(self, name):
        self.joint_name = name
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Get widget dimensions
        size = min(self.width(), self.height()) - 10
        center = QPoint(self.width() // 2, self.height() // 2)
        radius = size // 2
        
        # Draw outer circle
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(Qt.lightGray)
        painter.drawEllipse(center, radius, radius)
        
        # Draw inner circle
        painter.setBrush(Qt.white)
        painter.drawEllipse(center, radius - 10, radius - 10)
        
        # Draw scale marks
        painter.setPen(QPen(Qt.black, 1))
        for i in range(0, 181, 15):  # 0 to 180 degrees, every 15 degrees
            angle = math.radians(i - 90)  # Convert to radians, offset by 90 degrees
            inner_x = center.x() + (radius - 15) * math.cos(angle)
            inner_y = center.y() + (radius - 15) * math.sin(angle)
            outer_x = center.x() + (radius - 5) * math.cos(angle)
            outer_y = center.y() + (radius - 5) * math.sin(angle)
            painter.drawLine(QPoint(inner_x, inner_y), QPoint(outer_x, outer_y))
            
            # Draw scale numbers
            if i % 45 == 0:  # Every 45 degrees
                number = int((i / 180.0) * self.max_speed)
                text_x = center.x() + (radius - 30) * math.cos(angle)
                text_y = center.y() + (radius - 30) * math.sin(angle)
                painter.setFont(QFont("Arial", 8))
                painter.drawText(QPoint(text_x - 10, text_y + 5), str(number))
        
        # Draw speed needle
        if self.max_speed > 0:
            needle_angle = (self.current_speed / self.max_speed) * 180 - 90
            needle_rad = math.radians(needle_angle)
            needle_length = radius - 20
            needle_x = center.x() + needle_length * math.cos(needle_rad)
            needle_y = center.y() + needle_length * math.sin(needle_rad)
            
            # Color needle based on speed
            if self.current_speed / self.max_speed > 0.8:
                painter.setPen(QPen(Qt.red, 3))
            elif self.current_speed / self.max_speed > 0.5:
                painter.setPen(QPen(Qt.yellow, 3))
            else:
                painter.setPen(QPen(Qt.green, 3))
                
            painter.drawLine(center, QPoint(needle_x, needle_y))
            
            # Draw center circle
            painter.setPen(QPen(Qt.black, 1))
            painter.setBrush(Qt.black)
            painter.drawEllipse(center, 5, 5)
        
        # Draw speed text
        painter.setPen(Qt.black)
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        speed_text = f"{self.current_speed:.1f}"
        text_rect = QRect(center.x() - 30, center.y() + 10, 60, 20)
        painter.drawText(text_rect, Qt.AlignCenter, speed_text)
        
        # Draw joint name
        painter.setFont(QFont("Arial", 8))
        name_rect = QRect(center.x() - 40, center.y() - 30, 80, 15)
        painter.drawText(name_rect, Qt.AlignCenter, self.joint_name)


class Worker(QObject):
    """
    Worker for real-time updates to run in a separate thread.
    """
    update_signal = Signal(list) # Signal to send updated joint angles to GUI
    fk_update_signal = Signal(tuple, tuple) # Signal to send FK results to GUI

    def __init__(self, sender: rokisim_sender.RoKiSimSender, num_joints: int, max_speeds: list):
        super().__init__()
        self.sender = sender
        self.num_joints = num_joints
        self.max_speeds = max_speeds
        self.running = True

        self.joint_angles = [0.0] * self.num_joints
        self.target_angles = [0.0] * self.num_joints
        self.last_sent_angles = [0.0] * self.num_joints
        self.UPDATE_INTERVAL = 0.05
        self.SEND_THRESHOLD = 0.1
        self.global_speed_factor = 1.0 # Default global speed factor

    def update_target_angle(self, index, value):
        self.target_angles[index] = value

    def set_global_speed_factor(self, factor):
        self.global_speed_factor = factor

    def _interpolate_angle(self, current, target, max_speed, dt):
        delta = target - current
        # Apply global speed factor here
        max_step = max_speed * dt * self.global_speed_factor
        if abs(delta) <= max_step:
            return target
        else:
            return current + (max_step if delta > 0 else -max_step)

    def run(self):
        while self.running:
            start_time = time.perf_counter()
            has_significant_change = False

            for i in range(self.num_joints):
                new_angle = self._interpolate_angle(
                    self.joint_angles[i],
                    self.target_angles[i],
                    self.max_speeds[i],
                    self.UPDATE_INTERVAL
                )
                if new_angle != self.joint_angles[i]:
                    self.joint_angles[i] = new_angle
                    if abs(self.joint_angles[i] - self.last_sent_angles[i]) > self.SEND_THRESHOLD:
                        has_significant_change = True

            if has_significant_change and self.num_joints > 0:
                try:
                    self.sender.send_angles(self.joint_angles)
                    self.last_sent_angles[:] = self.joint_angles
                    self.update_signal.emit(self.joint_angles) # Emit signal to update sliders
                    logging.debug(f"Sent angles: {[round(a, 2) for a in self.joint_angles]}")
                except Exception as e:
                    logging.error(f"Failed to send angles in GUI loop: {e}")
            
            # Calculate FK in the worker thread and emit signal
            try:
                pos, rpy = self.sender.calculate_fk(self.joint_angles)
                if pos is not None and rpy is not None:
                    self.fk_update_signal.emit(pos, rpy)
            except Exception as e:
                logging.error(f"Error calculating FK in worker thread: {e}")

            elapsed_time = time.perf_counter() - start_time
            sleep_time = self.UPDATE_INTERVAL - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self.running = False


class JointControlGUI(QWidget):
    """
    PySide6 GUI for controlling robot joints.
    """
    def __init__(self, sender: rokisim_sender.RoKiSimSender):
        super().__init__()
        self.sender = sender
        self.robot_definition = None
        self.NUM_JOINTS = 0
        self.JOINT_MIN_ANGLES = []
        self.JOINT_MAX_ANGLES = []
        self.worker = None
        self.worker_thread = None
        self.last_joint_angles = [0.0] * 20  # Support up to 20 joints

        # Program execution variables
        self.compiler = EnhancedRobotCompiler(interpolation_steps=10)
        self.compiled_program = None
        self.program_running = False
        self.program_thread = None
        self.program_step_delay = 1.0  # Default step delay in seconds

        self.setWindowTitle("RoKiSim Joint Control - No Robot Loaded")
        self.max_speeds = []

        self.init_ui()
        # Don't start worker thread until a robot is loaded

        logging.info("GUI initialized without robot.")

    def init_ui(self):
        # Create tab widget
        self.tab_widget = QTabWidget()
        
        # Create joint control tab
        joint_tab = QWidget()
        joint_layout = QVBoxLayout(joint_tab)
        
        # Global Speed Control (disabled until robot is loaded)
        speed_group = QGroupBox("Global Speed Control")
        speed_layout = QHBoxLayout()
        speed_label = QLabel("Speed Factor:")
        self.global_speed_slider = QSlider(Qt.Horizontal)
        self.global_speed_slider.setMinimum(1) # 0.1
        self.global_speed_slider.setMaximum(60) # 2.0
        self.global_speed_slider.setValue(10) # 1.0
        self.global_speed_slider.setSingleStep(1)
        self.global_speed_slider.setTickInterval(1)
        self.global_speed_slider.setTickPosition(QSlider.TicksBelow)
        self.global_speed_slider.setEnabled(False)  # Disabled until robot loaded
        self.global_speed_value_label = QLabel("1.0x")
        self.global_speed_value_label.setFixedWidth(40)

        self.global_speed_slider.valueChanged.connect(self._on_global_speed_changed)

        speed_layout.addWidget(speed_label)
        speed_layout.addWidget(self.global_speed_slider)
        speed_layout.addWidget(self.global_speed_value_label)
        speed_group.setLayout(speed_layout)
        joint_layout.addWidget(speed_group)

        # Load XML Button
        load_xml_button = QPushButton("Load Robot XML")
        load_xml_button.clicked.connect(self._load_robot_xml)
        joint_layout.addWidget(load_xml_button)

        # Joint Control Sliders
        self.sliders = []
        self.joint_spinboxes = []  # For direct value editing
        joint_control_group = QGroupBox("Joint Control")
        
        # Use scroll area for joints to handle many joints
        scroll_area = QScrollArea()
        scroll_widget = QWidget()
        self.joint_layout = QVBoxLayout()
        self.joint_layout.setAlignment(Qt.AlignTop)

        # Initially show message that no robot is loaded
        self.no_robot_label = QLabel("No robot loaded. Please load an XML file to begin.")
        self.no_robot_label.setAlignment(Qt.AlignCenter)
        self.no_robot_label.setStyleSheet("color: gray; font-style: italic;")
        self.joint_layout.addWidget(self.no_robot_label)
        
        scroll_widget.setLayout(self.joint_layout)
        scroll_area.setWidget(scroll_widget)
        scroll_area.setWidgetResizable(True)
        scroll_area.setMinimumHeight(200)
        
        joint_control_layout = QVBoxLayout()
        joint_control_layout.addWidget(scroll_area)
        joint_control_group.setLayout(joint_control_layout)
        joint_layout.addWidget(joint_control_group)

        # FK Display (disabled until robot is loaded)
        self.fk_group = QGroupBox("Forward Kinematics")
        fk_layout = QHBoxLayout()
        self.fk_labels = []
        coords = ["X (mm):", "Y (mm):", "Z (mm):", "Roll (deg):", "Pitch (deg):", "Yaw (deg):"]
        for coord in coords:
            col_layout = QVBoxLayout()
            col_layout.addWidget(QLabel(coord))
            value_label = QLabel("N/A")
            value_label.setFixedWidth(80)
            value_label.setStyleSheet("color: gray;")
            col_layout.addWidget(value_label)
            self.fk_labels.append(value_label)
            fk_layout.addLayout(col_layout)
        self.fk_group.setLayout(fk_layout)
        self.fk_group.setEnabled(False)  # Disabled until robot loaded
        joint_layout.addWidget(self.fk_group)

        # IK Input (disabled until robot is loaded)
        self.ik_group = QGroupBox("Inverse Kinematics")
        ik_layout = QHBoxLayout()
        self.ik_entries = []
        ik_coords = ["X:", "Y:", "Z:", "R:", "P:", "Y:"]
        validator = QDoubleValidator()
        for coord in ik_coords:
            col_layout = QVBoxLayout()
            col_layout.addWidget(QLabel(coord))
            entry = QLineEdit()
            entry.setValidator(validator)
            entry.setFixedWidth(60)
            entry.setEnabled(False)  # Disabled until robot loaded
            self.ik_entries.append(entry)
            col_layout.addWidget(entry)
            ik_layout.addLayout(col_layout)

        self.ik_button = QPushButton("Move to Pose (IK)")
        self.ik_button.clicked.connect(self._on_calculate_ik)
        self.ik_button.setEnabled(False)  # Disabled until robot loaded
        ik_layout.addWidget(self.ik_button)

        self.ik_status_label = QLabel("")
        ik_layout.addWidget(self.ik_status_label)

        self.ik_group.setLayout(ik_layout)
        self.ik_group.setEnabled(False)  # Disabled until robot loaded
        joint_layout.addWidget(self.ik_group)

        # Add joint control tab
        self.tab_widget.addTab(joint_tab, "Joint Control")
        
        # Create program editor tab
        program_tab = QWidget()
        program_layout = QVBoxLayout(program_tab)
        
        # Program editor
        program_editor_group = QGroupBox("Robot Program")
        program_editor_layout = QVBoxLayout()
        
        self.program_editor = QPlainTextEdit()
        self.program_editor.setPlaceholderText("Enter your robot program here...\n\nExample:\nSETE HOME, [0,0,0,0,0,0]\nSETE P1, [10,20,30,40,50,60]\nMOVJ HOME\nMOVL P1\nFOR i, 1, 3\n  MOVJ P1\n  MOVJ HOME\nNEXT i")
        program_editor_layout.addWidget(self.program_editor)
        
        # Program execution speed control
        speed_control_layout = QHBoxLayout()
        speed_control_label = QLabel("Step Delay (s):")
        self.step_delay_spinbox = QDoubleSpinBox()
        self.step_delay_spinbox.setRange(0.1, 5.0)
        self.step_delay_spinbox.setValue(1.0)
        self.step_delay_spinbox.setSingleStep(0.1)
        self.step_delay_spinbox.setEnabled(False)
        self.step_delay_spinbox.valueChanged.connect(self._on_step_delay_changed)
        
        speed_control_layout.addWidget(speed_control_label)
        speed_control_layout.addWidget(self.step_delay_spinbox)
        speed_control_layout.addStretch()
        
        program_editor_layout.addLayout(speed_control_layout)
        
        # Buttons for program control
        button_layout = QHBoxLayout()
        self.compile_button = QPushButton("Compile")
        self.compile_button.clicked.connect(self.compile_program)
        self.compile_button.setEnabled(False)
        button_layout.addWidget(self.compile_button)
        
        self.run_button = QPushButton("Run")
        self.run_button.clicked.connect(self.run_program)
        self.run_button.setEnabled(False)
        button_layout.addWidget(self.run_button)
        
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_program)
        self.stop_button.setEnabled(False)
        button_layout.addWidget(self.stop_button)
        
        program_editor_layout.addLayout(button_layout)
        
        # Status label
        self.program_status_label = QLabel("No program compiled")
        program_editor_layout.addWidget(self.program_status_label)
        
        program_editor_group.setLayout(program_editor_layout)
        program_layout.addWidget(program_editor_group)
        
        # Program output display
        output_group = QGroupBox("Program Output")
        output_layout = QVBoxLayout()
        self.output_display = QPlainTextEdit()
        self.output_display.setReadOnly(True)
        output_layout.addWidget(self.output_display)
        output_group.setLayout(output_layout)
        program_layout.addWidget(output_group)
        
        # Add program editor tab
        self.tab_widget.addTab(program_tab, "Program Editor")
        
        # Set main layout
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(self.tab_widget)

    def _reset_joint_to_default(self, index):
        """Reset a specific joint to its default value (0.0)"""
        if hasattr(self, 'sliders') and index < len(self.sliders):
            self.sliders[index].setValue(0)
            if self.worker:
                self.worker.update_target_angle(index, 0.0)
            if hasattr(self, 'joint_spinboxes') and index < len(self.joint_spinboxes):
                self.joint_spinboxes[index].setValue(0.0)

    def _on_slider_moved(self, index, value):
        """Handle when a slider is moved"""
        angle = value / 10.0
        # Update spinbox
        if hasattr(self, 'joint_spinboxes') and index < len(self.joint_spinboxes):
            self.joint_spinboxes[index].blockSignals(True)
            self.joint_spinboxes[index].setValue(angle)
            self.joint_spinboxes[index].blockSignals(False)
        # Update worker target
        if self.worker:
            self.worker.update_target_angle(index, angle)

    def _on_spinbox_changed(self, index, value):
        """Handle when a spinbox value is changed"""
        # Update slider
        if hasattr(self, 'sliders') and index < len(self.sliders):
            self.sliders[index].blockSignals(True)
            self.sliders[index].setValue(int(value * 10))
            self.sliders[index].blockSignals(False)
        # Update worker target
        if self.worker:
            self.worker.update_target_angle(index, value)

    def _on_step_delay_changed(self, value):
        """Handle when step delay is changed"""
        self.program_step_delay = value

    def _load_robot_xml(self):
        """Load a new robot XML definition file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open Robot XML", "", "XML Files (*.xml)"
        )
        if file_path:
            try:
                self.sender.load_robot_definition(file_path)
                self._reload_robot_definition()
                QMessageBox.information(self, "Success", f"Loaded robot from:\n{os.path.basename(file_path)}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load XML:\n{e}")

    def _reload_robot_definition(self):
        """Reload the robot definition and rebuild the UI"""
        # Stop the current worker if it exists
        if self.worker:
            self.worker.stop()
            if self.worker_thread and self.worker_thread.is_alive():
                self.worker_thread.join(timeout=1)
            self.worker = None
            self.worker_thread = None

        # Reload robot definition
        self.robot_definition = self.sender.get_loaded_robot_definition()
        self.NUM_JOINTS = self.robot_definition.get_num_joints()

        default_min, default_max = -180.0, 180.0
        joint_mins, joint_maxs = self.robot_definition.get_joint_limits()
        if not joint_mins:
            joint_mins = [default_min] * self.NUM_JOINTS
        if not joint_maxs:
            joint_maxs = [default_max] * self.NUM_JOINTS

        self.JOINT_MIN_ANGLES = joint_mins
        self.JOINT_MAX_ANGLES = joint_maxs

        # Update window title
        self.setWindowTitle(f"RoKiSim Joint Control - {self.robot_definition.name} ({self.NUM_JOINTS} Joints)")

        # Reinitialize max speeds
        self.max_speeds = [10.0] * self.NUM_JOINTS

        # Clear and rebuild the UI
        self._clear_ui()
        self._build_robot_ui()

        # Enable program compilation and step delay control
        self.compile_button.setEnabled(True)
        self.step_delay_spinbox.setEnabled(True)

        # Start worker thread now that we have a robot
        self.start_worker_thread()

    def _clear_ui(self):
        """Clear the current UI - FIXED: Properly remove all joint controls"""
        # Clear all references to old widgets
        self.sliders = []
        self.joint_spinboxes = []
        self.joint_speed_gauges = []
        
        # Remove all items from the joint layout
        while self.joint_layout.count():
            child = self.joint_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
            elif child.layout():
                # Recursively delete layout items
                self._clear_layout(child.layout())
                
        # Reset last joint angles
        self.last_joint_angles = [0.0] * max(20, self.NUM_JOINTS)

    def _clear_layout(self, layout):
        """Helper method to recursively clear a layout"""
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
            elif child.layout():
                self._clear_layout(child.layout())

    def _build_robot_ui(self):
        """Build the UI for the loaded robot"""
        # Add joint controls
        if self.NUM_JOINTS > 0:
            for i in range(self.NUM_JOINTS):
                min_val = self.JOINT_MIN_ANGLES[i]
                max_val = self.JOINT_MAX_ANGLES[i]

                row_layout = QHBoxLayout()
                label = QLabel(f"Joint {i + 1} ({min_val:.0f}°..{max_val:.0f}°)")
                row_layout.addWidget(label)

                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(int(min_val * 10))
                slider.setMaximum(int(max_val * 10))
                slider.setValue(0)
                slider.setSingleStep(1)
                slider.setPageStep(10)
                slider.setTickInterval(100)
                slider.setTickPosition(QSlider.TicksBelow)
                
                # Create proper closure for slider callback
                def make_slider_callback(idx):
                    return lambda value: self._on_slider_moved(idx, value)
                
                slider.valueChanged.connect(make_slider_callback(i))
                row_layout.addWidget(slider)
                self.sliders.append(slider)

                # Use QDoubleSpinBox for direct value editing
                value_spinbox = QDoubleSpinBox()
                value_spinbox.setRange(min_val, max_val)
                value_spinbox.setValue(0.0)
                value_spinbox.setSingleStep(1.0)
                value_spinbox.setDecimals(1)
                value_spinbox.setFixedWidth(80)
                
                # Create proper closure for spinbox callback
                def make_spinbox_callback(idx):
                    return lambda value: self._on_spinbox_changed(idx, value)
                
                value_spinbox.valueChanged.connect(make_spinbox_callback(i))
                row_layout.addWidget(value_spinbox)
                self.joint_spinboxes.append(value_spinbox)

                # Reset button for each joint
                reset_button = QPushButton("Reset")
                reset_button.setFixedWidth(60)
                reset_button.clicked.connect(lambda checked, idx=i: self._reset_joint_to_default(idx))
                row_layout.addWidget(reset_button)

                self.joint_layout.addLayout(row_layout)
        else:
            no_joints_label = QLabel("No joints defined in loaded robot definition.")
            self.joint_layout.addWidget(no_joints_label)
            
        # Enable UI elements
        self.global_speed_slider.setEnabled(True)
        
        # Enable FK display
        self.fk_group.setEnabled(True)
        for label in self.fk_labels:
            label.setStyleSheet("")  # Remove gray styling
            label.setText("0.00")    # Initialize with default values
            
        # Enable IK entries and button
        self.ik_group.setEnabled(True)
        for entry in self.ik_entries:
            entry.setEnabled(True)
            entry.setText("")  # Clear any previous values
        self.ik_button.setEnabled(True)
        
        # Clear status label
        self.ik_status_label.setText("")

    def start_worker_thread(self):
        if self.NUM_JOINTS > 0:
            self.worker = Worker(self.sender, self.NUM_JOINTS, self.max_speeds)
            self.worker_thread = threading.Thread(target=self.worker.run, daemon=True)
            self.worker.update_signal.connect(self.update_joint_sliders) # Connect signal to slot
            self.worker.fk_update_signal.connect(self.update_fk_display) # Connect FK signal
            self.worker_thread.start()

    def _on_global_speed_changed(self, value):
        if self.worker:
            speed_factor = value / 10.0
            self.worker.set_global_speed_factor(speed_factor)
            self.global_speed_value_label.setText(f"{speed_factor:.1f}x")

    def update_joint_sliders(self, angles):
        if hasattr(self, 'sliders') and hasattr(self, 'joint_spinboxes'):
            # Calculate speeds for display
            current_time = time.time()
            if not hasattr(self, '_last_speed_update_time'):
                self._last_speed_update_time = current_time
                # Initialize last angles
                for i in range(min(len(angles), len(self.last_joint_angles))):
                    self.last_joint_angles[i] = angles[i]
                return
            
            dt = current_time - self._last_speed_update_time
            if dt > 0.05:  # Update speed display at reasonable intervals (20 FPS)
                for i, angle in enumerate(angles):
                    if i < len(self.sliders) and i < len(self.joint_spinboxes):
                        # Update slider position (scaled back to int for QSlider)
                        self.sliders[i].blockSignals(True) # Block signals to prevent recursive calls
                        self.sliders[i].setValue(int(angle * 10))
                        self.sliders[i].blockSignals(False)
                        
                        # Update spinbox
                        self.joint_spinboxes[i].blockSignals(True)
                        self.joint_spinboxes[i].setValue(angle)
                        self.joint_spinboxes[i].blockSignals(False)
                        
                        # Update speed display if gauges exist
                        if hasattr(self, 'joint_speed_gauges') and i < len(self.joint_speed_gauges):
                            speed = abs(angle - self.last_joint_angles[i]) / dt
                            self.joint_speed_gauges[i].set_speed(speed)
                        
                        # Update last angle for next speed calculation
                        self.last_joint_angles[i] = angle
                
                self._last_speed_update_time = current_time

    def update_fk_display(self, pos, rpy):
        # Check if UI elements exist and are valid
        if not hasattr(self, 'fk_labels') or len(self.fk_labels) < 6:
            return
            
        try:
            self.fk_labels[0].setText(f"{pos[0]:.2f}")
            self.fk_labels[1].setText(f"{pos[1]:.2f}")
            self.fk_labels[2].setText(f"{pos[2]:.2f}")
            self.fk_labels[3].setText(f"{rpy[0]:.2f}")
            self.fk_labels[4].setText(f"{rpy[1]:.2f}")
            self.fk_labels[5].setText(f"{rpy[2]:.2f}")
        except RuntimeError:
            # Widget was deleted, ignore
            pass

    def _on_calculate_ik(self):
        if not self.worker:
            return
            
        try:
            target_values = []
            for entry in self.ik_entries:
                val_str = entry.text()
                if not val_str:
                    raise ValueError("All IK target fields must be filled.")
                target_values.append(float(val_str))

            if len(target_values) != 6:
                raise ValueError("Need 6 values for IK target.")

            target_pose = tuple(target_values)
            # Pass current joint angles as initial guess
            ik_solution = self.sender.calculate_ik(target_pose, initial_guess=self.worker.joint_angles)

            if ik_solution is not None:
                # Update worker's target angles, which will then update sliders via signal
                for i, angle in enumerate(ik_solution):
                    self.worker.update_target_angle(i, angle)
                self.ik_status_label.setText("Target Set via IK")
                self.ik_status_label.setStyleSheet("color: green")
                logging.info(f"IK solution set: {ik_solution}")
            else:
                self.ik_status_label.setText("IK Failed to Find Solution")
                self.ik_status_label.setStyleSheet("color: orange")
                logging.info(f"IK failed to find solution for target {target_pose}")

        except ValueError as e:
            self.ik_status_label.setText(f"Input Error: {e}")
            self.ik_status_label.setStyleSheet("color: red")
            logging.warning(f"IK Input Error: {e}")
        except Exception as e:
            self.ik_status_label.setText(f"Error: {e}")
            self.ik_status_label.setStyleSheet("color: red")
            logging.error(f"Error in IK button callback: {e}")

    def compile_program(self):
        """Compile the robot program"""
        program_text = self.program_editor.toPlainText()
        
        if not program_text.strip():
            self.program_status_label.setText("No program to compile")
            self.program_status_label.setStyleSheet("color: red")
            return
        
        try:
            # Reset the compiler
            self.compiler = EnhancedRobotCompiler(interpolation_steps=10)
            
            # Set current joint angles as initial position
            if hasattr(self, 'worker') and self.worker:
                self.compiler.current_pose = self.worker.joint_angles[:]
            
            # Compile the program
            self.compiled_program = self.compiler.compile(program_text)
            
            self.program_status_label.setText(f"Program compiled successfully. {len(self.compiled_program)} positions generated.")
            self.program_status_label.setStyleSheet("color: green")
            self.run_button.setEnabled(True)
            
            # Show compiled positions in output
            self.output_display.setPlainText("Compiled positions:\n")
            for i, pos in enumerate(self.compiled_program):
                self.output_display.appendPlainText(f"{i}: {[round(p, 2) for p in pos]}")
                
        except Exception as e:
            self.program_status_label.setText(f"Compilation error: {str(e)}")
            self.program_status_label.setStyleSheet("color: red")
            self.run_button.setEnabled(False)

    def run_program(self):
        """Run the compiled program"""
        if self.compiled_program is None:
            self.program_status_label.setText("No program to run. Compile first.")
            self.program_status_label.setStyleSheet("color: red")
            return
        
        if not hasattr(self, 'worker') or not self.worker:
            self.program_status_label.setText("No robot loaded. Load a robot first.")
            self.program_status_label.setStyleSheet("color: red")
            return
        
        self.program_running = True
        self.compile_button.setEnabled(False)
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.program_status_label.setText("Program running...")
        self.program_status_label.setStyleSheet("color: blue")
        
        # Start program execution in a separate thread
        self.program_thread = threading.Thread(target=self.execute_program, daemon=True)
        self.program_thread.start()

    def stop_program(self):
        """Stop the running program"""
        self.program_running = False
        if self.program_thread and self.program_thread.is_alive():
            self.program_thread.join(timeout=1.0)
        
        self.compile_button.setEnabled(True)
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.program_status_label.setText("Program stopped")
        self.program_status_label.setStyleSheet("color: orange")

    def execute_program(self):
        """Execute the compiled program step by step"""
        try:
            step_delay = self.program_step_delay  # Use the configured step delay
            
            for i, target_position in enumerate(self.compiled_program):
                if not self.program_running:
                    break
                    
                # Update status in GUI thread
                self.program_status_label.setText(f"Executing step {i+1}/{len(self.compiled_program)}")
                
                # Set target position for all joints
                for j in range(len(target_position)):
                    if j < len(self.worker.target_angles):
                        self.worker.update_target_angle(j, target_position[j])
                
                # Wait for movement to complete or timeout
                start_time = time.time()
                movement_complete = False
                
                while not movement_complete and (time.time() - start_time) < 5.0:
                    if not self.program_running:
                        break
                        
                    # Check if we're close to the target
                    movement_complete = True
                    for j in range(len(target_position)):
                        if j < len(self.worker.joint_angles):
                            error = abs(self.worker.joint_angles[j] - target_position[j])
                            if error > 1.0:  # 1 degree tolerance
                                movement_complete = False
                                break
                    
                    time.sleep(0.1)
                
                # Add a small delay between movements
                time.sleep(step_delay)
            
            # Update status when done
            if self.program_running:
                self.program_status_label.setText("Program completed successfully")
                self.program_status_label.setStyleSheet("color: green")
                    
        except Exception as e:
            self.program_status_label.setText(f"Program execution error: {str(e)}")
            self.program_status_label.setStyleSheet("color: red")
        
        # Reset button states
        self.compile_button.setEnabled(True)
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.program_running = False

    def closeEvent(self, event):
        logging.info("Closing GUI application...")
        
        # Stop program execution if running
        if self.program_running:
            self.stop_program()
        
        # Stop worker thread
        if self.worker:
            self.worker.stop()
            if self.worker_thread and self.worker_thread.is_alive():
                self.worker_thread.join(timeout=1)
        
        super().closeEvent(event)
        logging.info("GUI application closed.")


# --- Example usage if run directly ---
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

    sender = rokisim_sender.RoKiSimSender()

    app = QApplication(sys.argv)
    gui = JointControlGUI(sender)
    gui.resize(1000, 800)  # Set a larger default size
    gui.show()
    sys.exit(app.exec())