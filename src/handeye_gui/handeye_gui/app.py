import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
import time

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QProcess
from PyQt5.QtGui import QImage, QPixmap, QTextCursor
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QScrollArea,
    QPushButton,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray


@dataclass
class CommandDefinition:
    name: str
    command: str


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


class CommandRunner(QObject):
    output_signal = pyqtSignal(str)
    status_signal = pyqtSignal(str)

    def __init__(self, name: str, command: str, working_dir: Optional[Path] = None):
        super().__init__()
        self.name = name
        self.command = command
        self.working_dir = working_dir
        self.process = QProcess()
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._handle_output)
        self.process.started.connect(lambda: self.status_signal.emit("running"))
        self.process.finished.connect(self._handle_finished)
        self.process.errorOccurred.connect(self._handle_error)

    def start(self, command: str) -> None:
        if self.process.state() != QProcess.NotRunning:
            return
        self.command = command
        if self.working_dir is not None:
            self.process.setWorkingDirectory(str(self.working_dir))
        self.process.start("bash", ["-lc", self.command])

    def stop(self) -> None:
        if self.process.state() == QProcess.NotRunning:
            return
        self.process.terminate()
        if not self.process.waitForFinished(2000):
            self.process.kill()
            self.process.waitForFinished(2000)
        self.process.close()
        self.status_signal.emit("stopped")

    def _handle_output(self) -> None:
        text = bytes(self.process.readAllStandardOutput()).decode(errors="replace")
        if text:
            self.output_signal.emit(f"[{self.name}] {text}")

    def _handle_finished(self, exit_code: int, _status: QProcess.ExitStatus) -> None:
        self.status_signal.emit(f"stopped ({exit_code})")

    def _handle_error(self, error: QProcess.ProcessError) -> None:
        self.status_signal.emit(f"error ({error})")


class RosInterface(QObject):
    real_image_signal = pyqtSignal(QImage)
    sim_image_signal = pyqtSignal(QImage)
    marked_image_signal = pyqtSignal(QImage)
    status_signal = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.node = rclpy.create_node("handeye_gui")
        self.bridge = CvBridge()
        self.real_sub = None
        self.sim_sub = None
        self.marked_sub = None
        self.status_sub = self.node.create_subscription(
            Int64MultiArray,
            "/handeye_logger/status",
            self._status_cb,
            10,
        )

    def set_real_topic(self, topic: str) -> None:
        if self.real_sub is not None:
            self.node.destroy_subscription(self.real_sub)
        self.real_sub = self.node.create_subscription(Image, topic, self._real_cb, 10)
        self.status_signal.emit(f"Real image topic: {topic}")

    def set_sim_topic(self, topic: str) -> None:
        if self.sim_sub is not None:
            self.node.destroy_subscription(self.sim_sub)
        self.sim_sub = self.node.create_subscription(Image, topic, self._sim_cb, 10)
        self.status_signal.emit(f"Sim image topic: {topic}")

    def set_marked_topic(self, topic: str) -> None:
        if self.marked_sub is not None:
            self.node.destroy_subscription(self.marked_sub)
        self.marked_sub = self.node.create_subscription(Image, topic, self._marked_cb, 10)
        self.status_signal.emit(f"Marked image topic: {topic}")

    def shutdown(self) -> None:
        for sub in (self.real_sub, self.sim_sub, self.marked_sub, self.status_sub):
            if sub is not None:
                self.node.destroy_subscription(sub)
        self.real_sub = None
        self.sim_sub = None
        self.marked_sub = None
        self.status_sub = None

    def _status_cb(self, msg: Int64MultiArray) -> None:
        count = msg.data[0] if msg.data else 0
        stamp = msg.data[1] if len(msg.data) > 1 else 0
        self.status_signal.emit(f"handeye_logger/status: count={count}, stamp_us={stamp}")

    def _real_cb(self, msg: Image) -> None:
        image = self._to_qimage(msg)
        self.real_image_signal.emit(image)

    def _sim_cb(self, msg: Image) -> None:
        image = self._to_qimage(msg)
        self.sim_image_signal.emit(image)

    def _marked_cb(self, msg: Image) -> None:
        image = self._to_qimage(msg)
        self.marked_image_signal.emit(image)

    def _to_qimage(self, msg: Image) -> QImage:
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = np.ascontiguousarray(cv_image)
        if cv_image.ndim == 2:
            if cv_image.dtype != np.uint8:
                cv_image = cv_image.astype(np.float32)
                cv_min = float(np.min(cv_image))
                cv_max = float(np.max(cv_image))
                scale = 255.0 / (cv_max - cv_min) if cv_max > cv_min else 1.0
                cv_image = ((cv_image - cv_min) * scale).astype(np.uint8)
            height, width = cv_image.shape
            bytes_per_line = width
            qimage = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
            return qimage.copy()

        if cv_image.shape[2] == 3:
            encoding = msg.encoding.lower()
            if encoding.startswith("bgr"):
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            height, width, _ = cv_image.shape
            bytes_per_line = 3 * width
            qimage = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            return qimage.copy()

        height, width, _ = cv_image.shape
        bytes_per_line = 4 * width
        qimage = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGBA8888)
        return qimage.copy()


class CommandControl(QWidget):
    def __init__(
        self,
        command_def: CommandDefinition,
        log_output: Optional[QTextEdit],
        working_dir: Optional[Path] = None,
    ) -> None:
        super().__init__()
        self.command_def = command_def
        self.log_output = log_output
        self.runner = CommandRunner(
            command_def.name,
            command_def.command,
            working_dir=working_dir,
        )
        self.runner.output_signal.connect(self._append_log)
        self.runner.status_signal.connect(self._update_status)

        self.name_label = QLabel(command_def.name)
        self.command_edit = QLineEdit(command_def.command)
        self.command_edit.setMinimumWidth(360)
        self.status_label = QLabel("stopped")
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.start_button.setFixedWidth(64)
        self.stop_button.setFixedWidth(64)

        self.start_button.clicked.connect(self._start)
        self.stop_button.clicked.connect(self._stop)

        layout = QHBoxLayout()
        layout.addWidget(self.name_label)
        layout.addWidget(self.command_edit, stretch=1)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.status_label)
        self.setLayout(layout)

    def stop(self) -> None:
        self.runner.stop()

    def _start(self) -> None:
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.runner.start(self.command_edit.text())

    def _stop(self) -> None:
        self.runner.stop()

    def _append_log(self, text: str) -> None:
        if self.log_output is None:
            return
        self.log_output.moveCursor(QTextCursor.End)
        self.log_output.insertPlainText(text)
        self.log_output.moveCursor(QTextCursor.End)

    def _update_status(self, status: str) -> None:
        self.status_label.setText(status)
        running = status.startswith("running")
        self.start_button.setEnabled(not running)
        self.stop_button.setEnabled(running)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Hand-Eye Calibration Control Panel")

        self.ros_interface = RosInterface()
        self.ros_interface.real_image_signal.connect(self._update_real_image)
        self.ros_interface.sim_image_signal.connect(self._update_sim_image)
        self.ros_interface.marked_image_signal.connect(self._update_marked_image)
        self.ros_interface.status_signal.connect(self._append_status)

        self._last_real_image = None
        self._last_sim_image = None
        self._last_marked_image = None
        self._last_real_image_time: Optional[float] = None
        self._last_sim_image_time: Optional[float] = None
        self._last_marked_image_time: Optional[float] = None
        self._arm_connected = False

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setPlaceholderText("Streaming logs (frequent updates)")
        self.tag_log_output = QTextEdit()
        self.tag_log_output.setReadOnly(True)
        self.tag_log_output.setPlaceholderText("Tag localizer output (high frequency)")
        self.event_log_output = QTextEdit()
        self.event_log_output.setReadOnly(True)
        self.event_log_output.setPlaceholderText("Event logs (start/stop/status)")

        self.build_runner = CommandRunner(
            "build",
            "colcon build --symlink-install",
            working_dir=WORKSPACE_ROOT,
        )
        self.build_runner.output_signal.connect(self._append_log)
        self.build_runner.status_signal.connect(self._append_status)

        self.build_command_edit = QLineEdit("colcon build --symlink-install")
        self.build_button = QPushButton("Build")
        self.build_button.clicked.connect(self._run_build)

        build_layout = QHBoxLayout()
        build_layout.addWidget(QLabel("Build command"))
        build_layout.addWidget(self.build_command_edit, stretch=1)
        build_layout.addWidget(self.build_button)

        self.tbc_edit = QLineEdit("0 0 0 0 0 0 1")
        self.tbc_apply_button = QPushButton("Apply TBC")
        self.tbc_apply_button.clicked.connect(self._apply_tbc)

        tbc_layout = QHBoxLayout()
        tbc_layout.addWidget(QLabel("Base->Camera (TBC) [x y z qx qy qz qw]"))
        tbc_layout.addWidget(self.tbc_edit, stretch=1)
        tbc_layout.addWidget(self.tbc_apply_button)

        self.command_controls = []
        self.dynamic_consistency_control = None
        commands = [
            CommandDefinition("Kinect", "ros2 run kinect kinect_node"),
            CommandDefinition("Tag Localizer", "ros2 run apriltag_detector tag_localizer_node"),
            CommandDefinition("HandEye Logger", "ros2 run handeye_logger handeye_logger"),
            CommandDefinition("RTDE Action", "ros2 run rtde_controller rtde_action_node"),
            CommandDefinition(
                "Run Calibration Poses",
                "bash scripts/handeye/run_calib_poses.sh",
            ),
            CommandDefinition(
                "Solve Handeye",
                "bash scripts/handeye/run_solver.sh",
            ),
            CommandDefinition(
                "Verify Repeatability (ROS)",
                "ros2 run handeye_verify verify_repeatability",
            ),
            CommandDefinition(
                "Verify Touch (ROS)",
                "ros2 run handeye_verify verify_handeye_touch",
            ),
            CommandDefinition(
                "Verify Dynamic Consistency",
                "ros2 run handeye_verify verify_dynamic_consistency "
                "--csv handeye_samples.csv --tbc 0 0 0 0 0 0 1",
            ),
        ]

        command_group = QGroupBox("Workflow Controls")
        command_layout = QVBoxLayout()
        for command in commands:
            log_output = self.log_output
            if command.name == "Tag Localizer":
                log_output = self.tag_log_output
            control = CommandControl(command, log_output, working_dir=WORKSPACE_ROOT)
            if command.name == "Verify Dynamic Consistency":
                self.dynamic_consistency_control = control
            if command.name == "RTDE Action":
                control.runner.status_signal.connect(self._update_arm_status)
            self.command_controls.append(control)
            command_layout.addWidget(control)
        command_group.setLayout(command_layout)

        self.real_topic_edit = self._build_topic_selector(
            "/color/image_raw",
            ["/color/image_raw", "/depth/image_raw", "/depth/aligned_depth_to_color"],
        )
        self.sim_topic_edit = self._build_topic_selector(
            "/simulation/image",
            ["/simulation/image", "/sim/image_raw", "/sim/color/image_raw"],
        )
        self.marked_topic_edit = self._build_topic_selector(
            "/apriltag/image_marked",
            ["/apriltag/image_marked"],
        )
        self.real_apply_button = QPushButton("Apply")
        self.sim_apply_button = QPushButton("Apply")
        self.marked_apply_button = QPushButton("Apply")
        self.real_apply_button.clicked.connect(self._apply_real_topic)
        self.sim_apply_button.clicked.connect(self._apply_sim_topic)
        self.marked_apply_button.clicked.connect(self._apply_marked_topic)

        real_group = self._build_image_group("Real Image", self.real_topic_edit, self.real_apply_button)
        sim_group = self._build_image_group("Sim Image", self.sim_topic_edit, self.sim_apply_button)
        marked_group = self._build_image_group(
            "Image Marked",
            self.marked_topic_edit,
            self.marked_apply_button,
        )

        self.real_image_label = QLabel("No image")
        self.sim_image_label = QLabel("No image")
        self.marked_image_label = QLabel("No image")
        for label in (self.real_image_label, self.sim_image_label, self.marked_image_label):
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(240)
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        real_group.layout().addWidget(self.real_image_label)
        sim_group.layout().addWidget(self.sim_image_label)
        marked_group.layout().addWidget(self.marked_image_label)

        status_group = QGroupBox("Status")
        self.status_label = QLabel("handeye_logger/status: awaiting data")
        status_indicators = QHBoxLayout()
        self.camera_status_label = self._build_status_indicator("Camera", "disconnected")
        self.arm_status_label = self._build_status_indicator("Arm", "disconnected")
        status_indicators.addWidget(self.camera_status_label)
        status_indicators.addWidget(self.arm_status_label)
        status_indicators.addStretch(1)
        status_layout = QVBoxLayout()
        status_layout.addWidget(self.status_label)
        status_layout.addLayout(status_indicators)
        status_group.setLayout(status_layout)

        images_layout = QGridLayout()
        images_layout.addWidget(real_group, 0, 0)
        images_layout.addWidget(sim_group, 0, 1)
        images_layout.addWidget(marked_group, 1, 0, 1, 2)

        main_layout = QVBoxLayout()
        main_layout.addLayout(build_layout)
        main_layout.addLayout(tbc_layout)
        main_layout.addWidget(command_group)
        main_layout.addLayout(images_layout)
        main_layout.addWidget(status_group)
        main_layout.addWidget(QLabel("Event Logs"))
        main_layout.addWidget(self.event_log_output)
        main_layout.addWidget(QLabel("Tag Localizer Logs"))
        main_layout.addWidget(self.tag_log_output)
        main_layout.addWidget(QLabel("Streaming Logs"))
        main_layout.addWidget(self.log_output)

        container = QWidget()
        container.setLayout(main_layout)

        # Make the central content scrollable so long panels don't overflow.
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(container)
        self.setCentralWidget(scroll_area)

        self._apply_real_topic()
        self._apply_sim_topic()
        self._apply_marked_topic()

        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(10)
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._refresh_status_indicators)
        self.status_timer.start(500)

    def _build_topic_selector(self, default: str, options: List[str]) -> QComboBox:
        selector = QComboBox()
        selector.setEditable(True)
        selector.addItems(options)
        selector.setCurrentText(default)
        return selector

    def _build_image_group(self, title: str, topic_edit: QComboBox, button: QPushButton) -> QGroupBox:
        group = QGroupBox(title)
        layout = QVBoxLayout()
        topic_row = QHBoxLayout()
        topic_row.addWidget(QLabel("Topic"))
        topic_row.addWidget(topic_edit, stretch=1)
        topic_row.addWidget(button)
        layout.addLayout(topic_row)
        group.setLayout(layout)
        return group

    def _build_status_indicator(self, label: str, state: str) -> QLabel:
        indicator = QLabel()
        indicator.setAlignment(Qt.AlignCenter)
        indicator.setMinimumWidth(140)
        self._set_indicator_state(indicator, label, state)
        return indicator

    def _set_indicator_state(self, indicator: QLabel, label: str, state: str) -> None:
        state = state.lower()
        colors = {
            "connected": "#34c759",
            "warning": "#ff9500",
            "disconnected": "#ff3b30",
        }
        color = colors.get(state, "#8e8e93")
        indicator.setText(f"{label}: {state}")
        indicator.setStyleSheet(
            "QLabel {"
            f"background-color: {color};"
            "color: white;"
            "border-radius: 10px;"
            "padding: 4px 8px;"
            "font-weight: 600;"
            "}"
        )

    def _run_build(self) -> None:
        self.build_runner.start(self.build_command_edit.text())

    def _apply_tbc(self) -> None:
        if self.dynamic_consistency_control is None:
            return
        tbc_value = self.tbc_edit.text().strip()
        command = (
            "ros2 run handeye_verify verify_dynamic_consistency "
            f"--csv handeye_samples.csv --tbc {tbc_value}"
        )
        self.dynamic_consistency_control.command_edit.setText(command)
        self._append_status(f"TBC updated: {tbc_value}")

    def _apply_real_topic(self) -> None:
        self.ros_interface.set_real_topic(self.real_topic_edit.currentText())

    def _apply_sim_topic(self) -> None:
        self.ros_interface.set_sim_topic(self.sim_topic_edit.currentText())

    def _apply_marked_topic(self) -> None:
        self.ros_interface.set_marked_topic(self.marked_topic_edit.currentText())

    def _spin_ros(self) -> None:
        rclpy.spin_once(self.ros_interface.node, timeout_sec=0)

    def _append_log(self, text: str) -> None:
        self.log_output.moveCursor(QTextCursor.End)
        self.log_output.insertPlainText(text)
        self.log_output.moveCursor(QTextCursor.End)

    def _append_status(self, text: str) -> None:
        self.status_label.setText(text)
        self.event_log_output.moveCursor(QTextCursor.End)
        self.event_log_output.insertPlainText(f"[status] {text}\n")
        self.event_log_output.moveCursor(QTextCursor.End)

    def _update_real_image(self, image: QImage) -> None:
        self._last_real_image = image
        self._last_real_image_time = time.time()
        self._render_image(self.real_image_label, image)

    def _update_sim_image(self, image: QImage) -> None:
        self._last_sim_image = image
        self._last_sim_image_time = time.time()
        self._render_image(self.sim_image_label, image)

    def _update_marked_image(self, image: QImage) -> None:
        self._last_marked_image = image
        self._last_marked_image_time = time.time()
        self._render_image(self.marked_image_label, image)

    def _update_arm_status(self, status: str) -> None:
        self._arm_connected = status.startswith("running")

    def _refresh_status_indicators(self) -> None:
        now = time.time()
        real_recent = self._last_real_image_time is not None and (now - self._last_real_image_time) < 1.0
        if real_recent:
            self._set_indicator_state(self.camera_status_label, "Camera", "connected")
        else:
            self._set_indicator_state(self.camera_status_label, "Camera", "disconnected")
            if self._last_real_image is not None:
                self.real_image_label.setPixmap(QPixmap())
                self.real_image_label.setText("No image")
                self._last_real_image = None

        sim_recent = self._last_sim_image_time is not None and (now - self._last_sim_image_time) < 1.0
        if not sim_recent and self._last_sim_image is not None:
            self.sim_image_label.setPixmap(QPixmap())
            self.sim_image_label.setText("No image")
            self._last_sim_image = None

        marked_recent = (
            self._last_marked_image_time is not None and (now - self._last_marked_image_time) < 1.0
        )
        if not marked_recent and self._last_marked_image is not None:
            self.marked_image_label.setPixmap(QPixmap())
            self.marked_image_label.setText("No image")
            self._last_marked_image = None

        arm_state = "connected" if self._arm_connected else "disconnected"
        self._set_indicator_state(self.arm_status_label, "Arm", arm_state)

    def _render_image(self, label: QLabel, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(pixmap)

    def resizeEvent(self, event) -> None:
        if self._last_real_image is not None:
            self._render_image(self.real_image_label, self._last_real_image)
        if self._last_sim_image is not None:
            self._render_image(self.sim_image_label, self._last_sim_image)
        if self._last_marked_image is not None:
            self._render_image(self.marked_image_label, self._last_marked_image)
        super().resizeEvent(event)

    def closeEvent(self, event) -> None:
        self.shutdown()
        super().closeEvent(event)

    def shutdown(self) -> None:
        self.spin_timer.stop()
        self.status_timer.stop()
        self.build_runner.stop()
        for control in self.command_controls:
            control.stop()
        self.ros_interface.shutdown()
        self.ros_interface.node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(1200, 800)
    window.show()
    exit_code = app.exec_()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
