import sys
from dataclasses import dataclass

import numpy as np
import rclpy
from cv_bridge import CvBridge
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QProcess
from PyQt5.QtGui import QImage, QPixmap, QTextCursor
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
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


class CommandRunner(QObject):
    output_signal = pyqtSignal(str)
    status_signal = pyqtSignal(str)

    def __init__(self, name: str, command: str):
        super().__init__()
        self.name = name
        self.command = command
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
        self.process.start("bash", ["-lc", self.command])

    def stop(self) -> None:
        if self.process.state() == QProcess.NotRunning:
            return
        self.process.terminate()
        if not self.process.waitForFinished(2000):
            self.process.kill()

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
    status_signal = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.node = rclpy.create_node("handeye_gui")
        self.bridge = CvBridge()
        self.real_sub = None
        self.sim_sub = None
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
            height, width, _ = cv_image.shape
            bytes_per_line = 3 * width
            encoding = msg.encoding.lower()
            image_format = QImage.Format_RGB888 if encoding.startswith("rgb") else QImage.Format_BGR888
            qimage = QImage(cv_image.data, width, height, bytes_per_line, image_format)
            return qimage.copy()

        height, width, _ = cv_image.shape
        bytes_per_line = 4 * width
        qimage = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGBA8888)
        return qimage.copy()


class CommandControl(QWidget):
    def __init__(self, command_def: CommandDefinition, log_output: QTextEdit) -> None:
        super().__init__()
        self.command_def = command_def
        self.log_output = log_output
        self.runner = CommandRunner(command_def.name, command_def.command)
        self.runner.output_signal.connect(self._append_log)
        self.runner.status_signal.connect(self._update_status)

        self.name_label = QLabel(command_def.name)
        self.command_edit = QLineEdit(command_def.command)
        self.command_edit.setMinimumWidth(360)
        self.status_label = QLabel("stopped")
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)

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
        self.ros_interface.status_signal.connect(self._append_status)

        self._last_real_image = None
        self._last_sim_image = None

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)

        self.build_runner = CommandRunner("build", "colcon build --symlink-install")
        self.build_runner.output_signal.connect(self._append_log)
        self.build_runner.status_signal.connect(self._append_status)

        self.build_command_edit = QLineEdit("colcon build --symlink-install")
        self.build_button = QPushButton("Build")
        self.build_button.clicked.connect(self._run_build)

        build_layout = QHBoxLayout()
        build_layout.addWidget(QLabel("Build command"))
        build_layout.addWidget(self.build_command_edit, stretch=1)
        build_layout.addWidget(self.build_button)

        self.command_controls = []
        commands = [
            CommandDefinition("Kinect", "ros2 run kinect kinect_node"),
            CommandDefinition("Tag Localizer", "ros2 run apriltag_detector tag_localizer_node"),
            CommandDefinition("HandEye Logger", "ros2 run handeye_logger handeye_logger"),
            CommandDefinition("RTDE Action", "ros2 run rtde_controller rtde_action_node"),
        ]

        command_group = QGroupBox("Node Launchers")
        command_layout = QVBoxLayout()
        for command in commands:
            control = CommandControl(command, self.log_output)
            self.command_controls.append(control)
            command_layout.addWidget(control)
        command_group.setLayout(command_layout)

        self.real_topic_edit = QLineEdit("/color/image_raw")
        self.sim_topic_edit = QLineEdit("/simulation/image")
        self.real_apply_button = QPushButton("Apply")
        self.sim_apply_button = QPushButton("Apply")
        self.real_apply_button.clicked.connect(self._apply_real_topic)
        self.sim_apply_button.clicked.connect(self._apply_sim_topic)

        real_group = self._build_image_group("Real Image", self.real_topic_edit, self.real_apply_button)
        sim_group = self._build_image_group("Sim Image", self.sim_topic_edit, self.sim_apply_button)

        self.real_image_label = QLabel("No image")
        self.sim_image_label = QLabel("No image")
        for label in (self.real_image_label, self.sim_image_label):
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(240)
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        real_group.layout().addWidget(self.real_image_label)
        sim_group.layout().addWidget(self.sim_image_label)

        status_group = QGroupBox("Status")
        self.status_label = QLabel("handeye_logger/status: awaiting data")
        status_layout = QVBoxLayout()
        status_layout.addWidget(self.status_label)
        status_group.setLayout(status_layout)

        images_layout = QGridLayout()
        images_layout.addWidget(real_group, 0, 0)
        images_layout.addWidget(sim_group, 0, 1)

        main_layout = QVBoxLayout()
        main_layout.addLayout(build_layout)
        main_layout.addWidget(command_group)
        main_layout.addLayout(images_layout)
        main_layout.addWidget(status_group)
        main_layout.addWidget(QLabel("Logs"))
        main_layout.addWidget(self.log_output)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        self._apply_real_topic()
        self._apply_sim_topic()

        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(10)

    def _build_image_group(self, title: str, topic_edit: QLineEdit, button: QPushButton) -> QGroupBox:
        group = QGroupBox(title)
        layout = QVBoxLayout()
        topic_row = QHBoxLayout()
        topic_row.addWidget(QLabel("Topic"))
        topic_row.addWidget(topic_edit, stretch=1)
        topic_row.addWidget(button)
        layout.addLayout(topic_row)
        group.setLayout(layout)
        return group

    def _run_build(self) -> None:
        self.build_runner.start(self.build_command_edit.text())

    def _apply_real_topic(self) -> None:
        self.ros_interface.set_real_topic(self.real_topic_edit.text())

    def _apply_sim_topic(self) -> None:
        self.ros_interface.set_sim_topic(self.sim_topic_edit.text())

    def _spin_ros(self) -> None:
        rclpy.spin_once(self.ros_interface.node, timeout_sec=0)

    def _append_log(self, text: str) -> None:
        self.log_output.moveCursor(QTextCursor.End)
        self.log_output.insertPlainText(text)
        self.log_output.moveCursor(QTextCursor.End)

    def _append_status(self, text: str) -> None:
        self.status_label.setText(text)
        self._append_log(f"[status] {text}\n")

    def _update_real_image(self, image: QImage) -> None:
        self._last_real_image = image
        self._render_image(self.real_image_label, image)

    def _update_sim_image(self, image: QImage) -> None:
        self._last_sim_image = image
        self._render_image(self.sim_image_label, image)

    def _render_image(self, label: QLabel, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(pixmap)

    def resizeEvent(self, event) -> None:
        if self._last_real_image is not None:
            self._render_image(self.real_image_label, self._last_real_image)
        if self._last_sim_image is not None:
            self._render_image(self.sim_image_label, self._last_sim_image)
        super().resizeEvent(event)

    def closeEvent(self, event) -> None:
        self.shutdown()
        super().closeEvent(event)

    def shutdown(self) -> None:
        self.spin_timer.stop()
        self.build_runner.stop()
        for control in self.command_controls:
            control.stop()
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
