import logging
import math
import os
import sys
import threading
import time
from typing import List, Optional, Tuple

from PySide6.QtCore import QObject, QPoint, QRect, Qt, Signal
from PySide6.QtGui import QColor, QDoubleValidator, QFont, QPainter, QPen
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
)

from .instruction_set import InstructionSetCompiler
from .sender import RoKiSimSender


class SpeedGauge(QWidget):
    """Circular gauge widget to display joint speed."""

    def __init__(self, max_speed: float = 50.0, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(120, 120)
        self.max_speed = max_speed
        self.current_speed = 0.0
        self.joint_name = "Joint"

    def set_speed(self, speed: float) -> None:
        self.current_speed = min(abs(speed), self.max_speed)
        self.update()

    def set_joint_name(self, name: str) -> None:
        self.joint_name = name
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        size = min(self.width(), self.height()) - 10
        center = QPoint(self.width() // 2, self.height() // 2)
        radius = size // 2
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(Qt.lightGray)
        painter.drawEllipse(center, radius, radius)
        painter.setBrush(Qt.white)
        painter.drawEllipse(center, radius - 10, radius - 10)
        painter.setPen(QPen(Qt.black, 1))
        for i in range(0, 181, 15):
            angle = math.radians(i - 90)
            inner_x = center.x() + (radius - 15) * math.cos(angle)
            inner_y = center.y() + (radius - 15) * math.sin(angle)
            outer_x = center.x() + (radius - 5) * math.cos(angle)
            outer_y = center.y() + (radius - 5) * math.sin(angle)
            painter.drawLine(QPoint(inner_x, inner_y), QPoint(outer_x, outer_y))
            if i % 45 == 0:
                number = int((i / 180.0) * self.max_speed)
                text_x = center.x() + (radius - 30) * math.cos(angle)
                text_y = center.y() + (radius - 30) * math.sin(angle)
                painter.setFont(QFont("Arial", 8))
                painter.drawText(QPoint(text_x - 10, text_y + 5), str(number))
        if self.max_speed > 0:
            needle_angle = (self.current_speed / self.max_speed) * 180 - 90
            needle_rad = math.radians(needle_angle)
            needle_length = radius - 20
            needle_x = center.x() + needle_length * math.cos(needle_rad)
            needle_y = center.y() + needle_length * math.sin(needle_rad)
            color = (
                Qt.red
                if self.current_speed / self.max_speed > 0.8
                else (
                    Qt.yellow if self.current_speed / self.max_speed > 0.5 else Qt.green
                )
            )
            painter.setPen(QPen(color, 3))
            painter.drawLine(center, QPoint(needle_x, needle_y))
            painter.setPen(QPen(Qt.black, 1))
            painter.setBrush(Qt.black)
            painter.drawEllipse(center, 5, 5)
        painter.setPen(Qt.black)
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        speed_text = f"{self.current_speed:.1f}"
        text_rect = QRect(center.x() - 30, center.y() + 10, 60, 20)
        painter.drawText(text_rect, Qt.AlignCenter, speed_text)
        painter.setFont(QFont("Arial", 8))
        name_rect = QRect(center.x() - 40, center.y() - 30, 80, 15)
        painter.drawText(name_rect, Qt.AlignCenter, self.joint_name)


class Worker(QObject):
    update_signal = Signal(list)
    fk_update_signal = Signal(tuple, tuple)

    def __init__(self, sender: RoKiSimSender, num_joints: int, max_speeds: List[float]):
        super().__init__()
        self.sender = sender
        self.num_joints = num_joints
        self.max_speeds = max_speeds
        self.running = True
        self.joint_angles: List[float] = [0.0] * num_joints
        self.target_angles: List[float] = [0.0] * num_joints
        self.last_sent_angles: List[float] = [0.0] * num_joints
        self.UPDATE_INTERVAL = 0.05
        self.SEND_THRESHOLD = 0.1
        self.global_speed_factor = 1.0

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
            has_change = False
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
                    self.update_signal.emit(self.joint_angles[:])
                    pos, rpy = self.sender.calculate_fk(self.joint_angles)
                    if pos and rpy:
                        self.fk_update_signal.emit(pos, rpy)
                except Exception as e:
                    logging.error(f"Error in worker: {e}")
            time.sleep(self.UPDATE_INTERVAL)

    def stop(self) -> None:
        self.running = False


class JointControlGUI(QWidget):
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
        self.setWindowTitle("RoKiSim Revival GUI")
        self.tabs = QTabWidget(self)
        layout = QVBoxLayout(self)
        layout.addWidget(self.tabs)
        self.setLayout(layout)
        self._setup_joint_tab()
        self._setup_program_tab()

    def _setup_joint_tab(self) -> None:
        joint_tab = QWidget()
        layout = QVBoxLayout(joint_tab)
        self.load_button = QPushButton("Load Robot XML")
        self.load_button.clicked.connect(self.load_robot_xml)
        layout.addWidget(self.load_button)
        self.joint_controls_group = QGroupBox("Joint Controls")
        self.joint_controls_layout = QVBoxLayout()
        self.joint_controls_group.setLayout(self.joint_controls_layout)
        scroll = QScrollArea()
        scroll.setWidget(self.joint_controls_group)
        scroll.setWidgetResizable(True)
        layout.addWidget(scroll)
        self.speed_factor_slider = QSlider(Qt.Horizontal)
        self.speed_factor_slider.setRange(1, 20)
        self.speed_factor_slider.setValue(10)
        self.speed_factor_slider.valueChanged.connect(self.update_global_speed)
        layout.addWidget(QLabel("Speed Factor (0.1x - 2.0x)"))
        layout.addWidget(self.speed_factor_slider)
        fk_group = QGroupBox("Forward Kinematics")
        fk_layout = QGridLayout()
        self.fk_labels = [QLabel() for _ in range(6)]
        labels = ["X:", "Y:", "Z:", "Roll:", "Pitch:", "Yaw:"]
        for i, label in enumerate(labels):
            fk_layout.addWidget(QLabel(label), i, 0)
            fk_layout.addWidget(self.fk_labels[i], i, 1)
        fk_group.setLayout(fk_layout)
        layout.addWidget(fk_group)
        ik_group = QGroupBox("Inverse Kinematics")
        ik_layout = QGridLayout()
        self.ik_entries = [QLineEdit() for _ in range(6)]
        for i, entry in enumerate(self.ik_entries):
            ik_layout.addWidget(QLabel(labels[i]), i, 0)
            ik_layout.addWidget(entry, i, 1)
            entry.setValidator(QDoubleValidator())
        self.ik_button = QPushButton("Move to Pose (IK)")
        self.ik_button.clicked.connect(self.move_to_ik_pose)
        ik_layout.addWidget(self.ik_button, 6, 0, 1, 2)
        self.ik_status_label = QLabel("")
        ik_layout.addWidget(self.ik_status_label, 7, 0, 1, 2)
        ik_group.setLayout(ik_layout)
        layout.addWidget(ik_group)
        self.tabs.addTab(joint_tab, "Joint Control")

    def _setup_program_tab(self) -> None:
        program_tab = QWidget()
        layout = QVBoxLayout(program_tab)
        self.program_editor = QPlainTextEdit()
        layout.addWidget(QLabel("Program Editor"))
        layout.addWidget(self.program_editor)
        buttons_layout = QHBoxLayout()
        self.compile_button = QPushButton("Compile")
        self.compile_button.clicked.connect(self.compile_program)
        buttons_layout.addWidget(self.compile_button)
        self.run_button = QPushButton("Run")
        self.run_button.clicked.connect(self.run_program)
        self.run_button.setEnabled(False)
        buttons_layout.addWidget(self.run_button)
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_program)
        self.stop_button.setEnabled(False)
        buttons_layout.addWidget(self.stop_button)
        layout.addLayout(buttons_layout)
        self.program_status_label = QLabel("")
        layout.addWidget(self.program_status_label)
        self.output_display = QPlainTextEdit()
        self.output_display.setReadOnly(True)
        layout.addWidget(QLabel("Output"))
        layout.addWidget(self.output_display)
        self.tabs.addTab(program_tab, "Program Editor")

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
                QMessageBox.information(
                    self, "Success", f"Loaded robot with {num_joints} joints."
                )
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load XML: {e}")

    def _create_joint_controls(self, num_joints: int) -> None:
        for i in reversed(range(self.joint_controls_layout.count())):
            widget = self.joint_controls_layout.itemAt(i).widget()
            if widget:
                widget.deleteLater()
        self.sliders: List[QSlider] = []
        self.spin_boxes: List[QDoubleSpinBox] = []
        self.gauges: List[SpeedGauge] = []
        self.reset_buttons: List[QPushButton] = []
        min_angles, max_angles = self.sender.robot_definition.get_joint_limits()
        for i in range(num_joints):
            joint_layout = QHBoxLayout()
            label = QLabel(f"Joint {i+1} [{min_angles[i]:.1f} to {max_angles[i]:.1f}]")
            joint_layout.addWidget(label)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(min_angles[i] * 10), int(max_angles[i] * 10))
            slider.setValue(0)
            slider.valueChanged.connect(
                lambda v, idx=i: self.update_target_from_slider(idx, v / 10.0)
            )
            self.sliders.append(slider)
            joint_layout.addWidget(slider)
            spin_box = QDoubleSpinBox()
            spin_box.setRange(min_angles[i], max_angles[i])
            spin_box.setValue(0.0)
            spin_box.valueChanged.connect(
                lambda v, idx=i: self.update_target_from_spin(idx, v)
            )
            self.spin_boxes.append(spin_box)
            joint_layout.addWidget(spin_box)
            reset_button = QPushButton("Reset")
            reset_button.clicked.connect(lambda _, idx=i: self.reset_joint(idx))
            self.reset_buttons.append(reset_button)
            joint_layout.addWidget(reset_button)
            gauge = SpeedGauge()
            gauge.set_joint_name(f"J{i+1}")
            self.gauges.append(gauge)
            joint_layout.addWidget(gauge)
            self.joint_controls_layout.addLayout(joint_layout)

    def _start_worker(self, num_joints: int) -> None:
        max_speeds = [50.0] * num_joints  # Arbitrary max speeds; configure if needed
        self.worker = Worker(self.sender, num_joints, max_speeds)
        self.worker.update_signal.connect(self.update_gui_from_worker)
        self.worker.fk_update_signal.connect(self.update_fk_display)
        self.worker_thread = threading.Thread(target=self.worker.run, daemon=True)
        self.worker_thread.start()

    def update_target_from_slider(self, index: int, value: float) -> None:
        if self.worker:
            self.worker.update_target_angle(index, value)
            self.spin_boxes[index].setValue(value)

    def update_target_from_spin(self, index: int, value: float) -> None:
        if self.worker:
            self.worker.update_target_angle(index, value)
            self.sliders[index].setValue(int(value * 10))

    def reset_joint(self, index: int) -> None:
        if self.worker:
            self.worker.update_target_angle(index, 0.0)
            self.spin_boxes[index].setValue(0.0)
            self.sliders[index].setValue(0)

    def update_global_speed(self, value: int) -> None:
        factor = value / 10.0
        if self.worker:
            self.worker.set_global_speed_factor(factor)

    def update_gui_from_worker(self, angles: List[float]) -> None:
        for i, angle in enumerate(angles):
            self.sliders[i].setValue(int(angle * 10))
            self.spin_boxes[i].setValue(angle)
            # Update gauge with speed (simplified; calculate delta if needed)
            self.gauges[i].set_speed(0.0)  # Placeholder; compute actual speed

    def update_fk_display(self, pos: Tuple[float, ...], rpy: Tuple[float, ...]) -> None:
        values = list(pos) + list(rpy)
        for i, label in enumerate(self.fk_labels):
            label.setText(f"{values[i]:.2f}")

    def move_to_ik_pose(self) -> None:
        try:
            # First, check if all fields are non-empty (and optionally non-whitespace)
            if any(not entry.text().strip() for entry in self.ik_entries):
                raise ValueError("Empty field")

            # Then, safely convert to floats (will still raise ValueError if non-numeric)
            target_values = [float(entry.text()) for entry in self.ik_entries]
            target_pose = tuple(target_values)
            ik_solution = self.sender.calculate_ik(
                target_pose, self.worker.joint_angles if self.worker else None
            )
            if ik_solution:
                for i, angle in enumerate(ik_solution):
                    self.update_target_from_spin(i, angle)
                self.ik_status_label.setText("IK Success")
                self.ik_status_label.setStyleSheet("color: green")
            else:
                self.ik_status_label.setText("IK Failed")
                self.ik_status_label.setStyleSheet("color: red")
        except ValueError as e:
            self.ik_status_label.setText(f"Error: {e}")
            self.ik_status_label.setStyleSheet("color: red")

    def compile_program(self) -> None:
        program_text = self.program_editor.toPlainText()
        if not program_text.strip():
            self.program_status_label.setText("No program")
            self.program_status_label.setStyleSheet("color: red")
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
            self.program_status_label.setStyleSheet("color: green")
            self.run_button.setEnabled(True)
            self.output_display.setPlainText(
                "Compiled positions:\n"
                + "\n".join(
                    f"{i}: {pos}" for i, pos in enumerate(self.compiled_program)
                )
            )
        except Exception as e:
            self.program_status_label.setText(f"Error: {e}")
            self.program_status_label.setStyleSheet("color: red")
            self.run_button.setEnabled(False)

    def run_program(self) -> None:
        if not self.compiled_program or not self.worker:
            self.program_status_label.setText("No program or robot")
            self.program_status_label.setStyleSheet("color: red")
            return
        self.program_running = True
        self.compile_button.setEnabled(False)
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.program_status_label.setText("Running...")
        self.program_status_label.setStyleSheet("color: blue")
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
        self.program_status_label.setStyleSheet("color: orange")

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
                self.program_status_label.setStyleSheet("color: green")
        except Exception as e:
            self.program_status_label.setText(f"Error: {e}")
            self.program_status_label.setStyleSheet("color: red")
        self.compile_button.setEnabled(True)
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.program_running = False

    def closeEvent(self, event) -> None:
        if self.program_running:
            self.stop_program()
        if self.worker:
            self.worker.stop()
            if self.worker_thread and self.worker_thread.is_alive():
                self.worker_thread.join(timeout=1)
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    sender = RoKiSimSender()
    gui = JointControlGUI(sender)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
