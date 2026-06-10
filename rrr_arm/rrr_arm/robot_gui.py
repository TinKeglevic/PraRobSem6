#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QLineEdit, QPushButton, QTabWidget, QGroupBox,
    QTextEdit, QGridLayout
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject
from PyQt5.QtGui import QFont


class RobotSignals(QObject):
    joint_states_received  = pyqtSignal(list)
    target_states_received = pyqtSignal(list)
    objects_detected       = pyqtSignal(str)


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.signals = RobotSignals()

        # Publisheri
        self.marker_pub = self.create_publisher(
            Point, '/marker_target', 10
        )
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )
        self.nlp_pub = self.create_publisher(
            String, '/nlp_command', 10
        )

        # Subscriberi
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10
        )
        self.create_subscription(
            JointState, '/joint_trajectory_controller/target_joint_states', self._on_target_states, 10
        )
        self.create_subscription(
            String, '/detected_objects', self._on_detected_objects, 10
        )

    def send_xyz(self, x, y, z):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.marker_pub.publish(msg)

    def send_joints(self, q1, q2, q3, duration_sec=1.0):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['joint1', 'joint2', 'joint3']
        point = JointTrajectoryPoint()
        point.positions = [float(q1), float(q2), float(q3)]
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        traj.points.append(point)
        self.joint_cmd_pub.publish(traj)

    def send_nlp_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.nlp_pub.publish(msg)

    def _on_joint_states(self, msg):
        if len(msg.position) >= 3:
            self.signals.joint_states_received.emit(list(msg.position[:3]))

    def _on_target_states(self, msg):
        if len(msg.position) >= 3:
            self.signals.target_states_received.emit(list(msg.position[:3]))

    def _on_detected_objects(self, msg):
        self.signals.objects_detected.emit(msg.data)


class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)


class RobotGUI(QMainWindow):
    def __init__(self, robot_node):
        super().__init__()
        self.robot_node = robot_node
        self.setWindowTitle("RRR Robot Controller")
        self.setMinimumSize(700, 600)

        self.robot_node.signals.joint_states_received.connect(self._on_joint_states)
        self.robot_node.signals.target_states_received.connect(self._on_target_states)
        self.robot_node.signals.objects_detected.connect(self._on_objects_detected)

        self._setup_style()
        self._build_ui()

    def _setup_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #1a1a2e; color: #e0e0e0; font-family: monospace; }
            QTabWidget::pane { border: 1px solid #333; border-radius: 4px; }
            QTabBar::tab { background: #16213e; color: #888; padding: 8px 20px; border-radius: 4px 4px 0 0; }
            QTabBar::tab:selected { background: #0f3460; color: #e0e0e0; }
            QGroupBox { border: 1px solid #333; border-radius: 6px; margin-top: 12px; padding: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; color: #4fc3f7; }
            QSlider::groove:horizontal { height: 4px; background: #333; border-radius: 2px; }
            QSlider::handle:horizontal { background: #4fc3f7; width: 14px; height: 14px; margin: -5px 0; border-radius: 7px; }
            QSlider::sub-page:horizontal { background: #4fc3f7; border-radius: 2px; }
            QLineEdit { background: #16213e; border: 1px solid #333; border-radius: 4px; padding: 6px; color: #e0e0e0; }
            QLineEdit:focus { border: 1px solid #4fc3f7; }
            QPushButton { background: #0f3460; border: none; border-radius: 4px; padding: 8px 16px; color: #e0e0e0; }
            QPushButton:hover { background: #4fc3f7; color: #1a1a2e; }
            QPushButton#send_btn { background: #1b5e20; font-weight: bold; padding: 10px; }
            QPushButton#send_btn:hover { background: #2e7d32; }
            QTextEdit { background: #0d0d1a; border: 1px solid #333; border-radius: 4px; color: #4fc3f7; font-family: monospace; }
            QLabel#status { color: #4fc3f7; font-size: 11px; }
        """)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(8)
        layout.setContentsMargins(12, 12, 12, 12)

        title = QLabel("RRR ROBOT CONTROLLER")
        title.setFont(QFont("monospace", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #4fc3f7; padding: 8px;")
        layout.addWidget(title)

        tabs = QTabWidget()
        tabs.addTab(self._build_dk_tab(), "Direktna kinematika")
        tabs.addTab(self._build_ik_tab(), "Inverzna kinematika")
        tabs.addTab(self._build_auto_tab(), "Autonomni mod")
        layout.addWidget(tabs)

        self.status_label = QLabel("Spreman.")
        self.status_label.setObjectName("status")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

    def _build_dk_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setSpacing(10)

        info = QLabel("Unesi kutove zglobova → publishа se na /joint_trajectory_controller/joint_trajectory → motori se pomaknu.")
        info.setStyleSheet("color: #888; font-size: 11px;")
        info.setWordWrap(True)
        layout.addWidget(info)

        joints_box = QGroupBox("Zglobovi (radijani)")
        jl = QGridLayout(joints_box)

        self.dk_sliders = []
        self.dk_inputs  = []
        self.dk_labels  = []

        for i, (name, lo, hi, default) in enumerate([
            ("joint1 (baza)",  -3.14, 3.14,  0.0),
            ("joint2 (rame)",  -3.14, 3.14,  0.5),
            ("joint3 (lakat)", -3.14, 3.14, -0.5),
        ]):
            jl.addWidget(QLabel(name), i, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(lo * 100))
            slider.setMaximum(int(hi * 100))
            slider.setValue(int(default * 100))
            jl.addWidget(slider, i, 1)

            val_input = QLineEdit(f"{default:.3f}")
            val_input.setFixedWidth(80)
            jl.addWidget(val_input, i, 2)

            val_lbl = QLabel(f"{default:.3f} rad")
            val_lbl.setFixedWidth(90)
            val_lbl.setStyleSheet("color: #4fc3f7;")
            jl.addWidget(val_lbl, i, 3)

            def on_slider(val, ii=i):
                rad = val / 100.0
                self.dk_inputs[ii].blockSignals(True)
                self.dk_inputs[ii].setText(f"{rad:.3f}")
                self.dk_inputs[ii].blockSignals(False)
                self.dk_labels[ii].setText(f"{rad:.3f} rad")
            slider.valueChanged.connect(on_slider)

            def on_input(text, ii=i):
                try:
                    rad = float(text)
                    self.dk_sliders[ii].blockSignals(True)
                    self.dk_sliders[ii].setValue(int(rad * 100))
                    self.dk_sliders[ii].blockSignals(False)
                    self.dk_labels[ii].setText(f"{rad:.3f} rad")
                except ValueError:
                    pass
            val_input.textChanged.connect(on_input)

            self.dk_sliders.append(slider)
            self.dk_inputs.append(val_input)
            self.dk_labels.append(val_lbl)

        layout.addWidget(joints_box)

        pos_box = QGroupBox("Stvarni kutovi (iz /joint_states)")
        pl = QHBoxLayout(pos_box)
        self.dk_x_lbl = QLabel("q1 = —")
        self.dk_y_lbl = QLabel("q2 = —")
        self.dk_z_lbl = QLabel("q3 = —")
        for l in [self.dk_x_lbl, self.dk_y_lbl, self.dk_z_lbl]:
            l.setStyleSheet("color: #4fc3f7; font-size: 13px;")
            l.setAlignment(Qt.AlignCenter)
            pl.addWidget(l)
        layout.addWidget(pos_box)

        btn = QPushButton("▶  Pošalji kutove na robot")
        btn.setObjectName("send_btn")
        btn.clicked.connect(self._send_dk)
        layout.addWidget(btn)
        layout.addStretch()
        return w

    def _send_dk(self):
        try:
            q1 = float(self.dk_inputs[0].text())
            q2 = float(self.dk_inputs[1].text())
            q3 = float(self.dk_inputs[2].text())
            self.robot_node.send_joints(q1, q2, q3)
            self._set_status(f"✓ Poslano: q1={q1:.3f}, q2={q2:.3f}, q3={q3:.3f}")
        except Exception as e:
            self._set_status(f"✗ Greška: {e}", error=True)

    def _build_ik_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setSpacing(10)

        info = QLabel("Unesi XYZ → publishа se na /marker_target → kinematics_node računa IK → kutovi dolaze na /joint_trajectory_controller/target_joint_states.")
        info.setStyleSheet("color: #888; font-size: 11px;")
        info.setWordWrap(True)
        layout.addWidget(info)

        pos_box = QGroupBox("Željena pozicija end-effektora (metri)")
        pl = QGridLayout(pos_box)
        self.ik_inputs = {}
        for i, (axis, default) in enumerate([("x", 0.15), ("y", 0.10), ("z", 0.09)]):
            pl.addWidget(QLabel(f"{axis} (m):"), i, 0)
            inp = QLineEdit(str(default))
            pl.addWidget(inp, i, 1)
            self.ik_inputs[axis] = inp
        layout.addWidget(pos_box)

        joints_box = QGroupBox("Izračunati kutovi (iz kinematics_node → /joint_trajectory_controller/target_joint_states)")
        jl = QHBoxLayout(joints_box)
        self.ik_q1_lbl = QLabel("joint1 = —")
        self.ik_q2_lbl = QLabel("joint2 = —")
        self.ik_q3_lbl = QLabel("joint3 = —")
        for l in [self.ik_q1_lbl, self.ik_q2_lbl, self.ik_q3_lbl]:
            l.setStyleSheet("color: #4fc3f7; font-size: 13px;")
            l.setAlignment(Qt.AlignCenter)
            jl.addWidget(l)
        layout.addWidget(joints_box)

        btn = QPushButton("▶  Pošalji poziciju na kinematics_node")
        btn.setObjectName("send_btn")
        btn.clicked.connect(self._send_ik)
        layout.addWidget(btn)
        layout.addStretch()
        return w

    def _send_ik(self):
        try:
            x = float(self.ik_inputs["x"].text())
            y = float(self.ik_inputs["y"].text())
            z = float(self.ik_inputs["z"].text())
            self.robot_node.send_xyz(x, y, z)
            self._set_status(f"✓ Poslano: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        except Exception as e:
            self._set_status(f"✗ Greška: {e}", error=True)

    def _build_auto_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setSpacing(10)

        info = QLabel(
            "Unesi naredbu → publishа se na /nlp_command → "
            "autonomous node detektira objekte i planira putanju."
        )
        info.setStyleSheet("color: #888; font-size: 11px;")
        info.setWordWrap(True)
        layout.addWidget(info)

        cmd_box = QGroupBox("Naredba")
        cl = QVBoxLayout(cmd_box)
        self.nlp_input = QLineEdit()
        self.nlp_input.setPlaceholderText("npr. 'spoji avion i auto, izbjegni loptu'")
        self.nlp_input.returnPressed.connect(self._send_autonomous)
        cl.addWidget(self.nlp_input)
        send_btn = QPushButton("▶  Izvrši naredbu")
        send_btn.setObjectName("send_btn")
        send_btn.clicked.connect(self._send_autonomous)
        cl.addWidget(send_btn)
        layout.addWidget(cmd_box)

        obj_box = QGroupBox("Detektirani objekti (iz /detected_objects)")
        ol = QVBoxLayout(obj_box)
        self.detected_label = QLabel("—")
        self.detected_label.setStyleSheet("color: #4fc3f7;")
        ol.addWidget(self.detected_label)
        layout.addWidget(obj_box)

        log_box = QGroupBox("Log")
        ll = QVBoxLayout(log_box)
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setMinimumHeight(150)
        ll.addWidget(self.log_output)
        clear_btn = QPushButton("Očisti log")
        clear_btn.clicked.connect(self.log_output.clear)
        ll.addWidget(clear_btn)
        layout.addWidget(log_box)

        layout.addStretch()
        return w

    def _send_autonomous(self):
        cmd = self.nlp_input.text().strip()
        if not cmd:
            self._set_status("✗ Unesi naredbu!", error=True)
            return
        self.robot_node.send_nlp_command(cmd)
        self.log_output.append(f">> {cmd}")
        self._set_status(f"▶ Naredba poslana: '{cmd}'")

    def _on_joint_states(self, angles):
        q1, q2, q3 = angles
        self.dk_x_lbl.setText(f"q1 = {math.degrees(q1):.1f}°")
        self.dk_y_lbl.setText(f"q2 = {math.degrees(q2):.1f}°")
        self.dk_z_lbl.setText(f"q3 = {math.degrees(q3):.1f}°")

    def _on_target_states(self, angles):
        q1, q2, q3 = angles
        self.ik_q1_lbl.setText(f"joint1 = {math.degrees(q1):.1f}°")
        self.ik_q2_lbl.setText(f"joint2 = {math.degrees(q2):.1f}°")
        self.ik_q3_lbl.setText(f"joint3 = {math.degrees(q3):.1f}°")

    def _on_objects_detected(self, data):
        self.detected_label.setText(data)
        self.log_output.append(f"   Detektirani: {data}")

    def _set_status(self, msg, error=False):
        color = "#ef5350" if error else "#4fc3f7"
        self.status_label.setStyleSheet(f"color: {color}; font-size: 11px;")
        self.status_label.setText(msg)


def main():
    rclpy.init()
    robot_node = RobotNode()

    ros_thread = RosThread(robot_node)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = RobotGUI(robot_node)
    gui.show()

    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()