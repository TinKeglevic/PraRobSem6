#!/usr/bin/env python3

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QLineEdit, QPushButton, QTabWidget, QGroupBox,
    QTextEdit, QGridLayout, QFrame
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor
import threading

# ─────────────────────────────────────────────
#  KINEMATIKA - ZAMIJENI VRIJEDNOSTI SUTRA!
# ─────────────────────────────────────────────
L1          = 0.173   # nadlaktica (os motor2 → os motor3)
L2          = 0.200   # podlaktica (os motor3 → vrh markera)
BASE_HEIGHT = 0.084   # visina od tla do osi motor1
OFFSET      = 0.015   # horizontalni pomak motor1 → motor2

def direct_kinematics(q1, q2, q3):
    """Iz kutova zglobova daj (x, y, z) end-effektora."""
    x1 = L1 * np.cos(q1) * np.cos(q2)
    y1 = L1 * np.sin(q1) * np.cos(q2)
    z1 = BASE_HEIGHT - L1 * np.sin(q2)

    x = x1 + L2 * np.cos(q1) * np.cos(q2 + q3)
    y = y1 + L2 * np.sin(q1) * np.cos(q2 + q3)
    z = z1 - L2 * np.sin(q2 + q3)
    return x, y, z

def inverse_kinematics(x, y, z):
    """Iz (x, y, z) end-effektora daj kutove zglobova."""
    planar_dist = np.sqrt(max(x**2 + y**2 - OFFSET**2, 0))
    d = np.sqrt(planar_dist**2 + BASE_HEIGHT**2)
    D = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    Q = (L1**2 + d**2 - L2**2) / (2 * L1 * d)

    if abs(D) > 1 or abs(Q) > 1:
        raise ValueError("Pozicija izvan dosega robota!")

    q3 = np.arccos(D) - 1.5708
    delta_fi = np.arcsin(OFFSET / np.sqrt(x**2 + y**2))
    q1 = np.arctan2(y, x) - delta_fi
    q2 = np.pi - np.arctan2(planar_dist, BASE_HEIGHT) - np.arccos(Q)
    return q1, q2, q3

# ─────────────────────────────────────────────
#  ROS2 NODE (radi u pozadini)
# ─────────────────────────────────────────────
class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

    def send_joints(self, q1, q2, q3, duration_sec=1.0):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3']
        point = JointTrajectoryPoint()
        point.positions = [float(q1), float(q2), float(q3)]
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        traj.points.append(point)
        self.publisher.publish(traj)

class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

# ─────────────────────────────────────────────
#  GLAVNI GUI
# ─────────────────────────────────────────────
class RobotGUI(QMainWindow):
    def __init__(self, robot_node):
        super().__init__()
        self.robot_node = robot_node
        self.setWindowTitle("Robot Controller")
        self.setMinimumSize(700, 600)
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
            QPushButton:pressed { background: #0288d1; }
            QPushButton#send_btn { background: #1b5e20; font-weight: bold; padding: 10px; }
            QPushButton#send_btn:hover { background: #2e7d32; }
            QTextEdit { background: #0d0d1a; border: 1px solid #333; border-radius: 4px; color: #4fc3f7; font-family: monospace; }
            QLabel#status { color: #4fc3f7; font-size: 11px; }
            QFrame#divider { background: #333; }
        """)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(12, 12, 12, 12)

        # Naslov
        title = QLabel("🤖 ROBOT CONTROLLER")
        title.setFont(QFont("monospace", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #4fc3f7; padding: 8px;")
        main_layout.addWidget(title)

        # Tabs
        tabs = QTabWidget()
        tabs.addTab(self._build_manual_dk_tab(), "Manual — Direktna kinematika")
        tabs.addTab(self._build_manual_ik_tab(), "Manual — Inverzna kinematika")
        tabs.addTab(self._build_autonomous_tab(), "Autonomni mod")
        main_layout.addWidget(tabs)

        # Status bar
        self.status_label = QLabel("Spreman.")
        self.status_label.setObjectName("status")
        self.status_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status_label)

    # ── TAB 1: Direktna kinematika ──────────────────────────────────────
    def _build_manual_dk_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setSpacing(10)

        info = QLabel("Unesi kutove zglobova → robot se pomiče → prikazuje se pozicija end-effektora.")
        info.setStyleSheet("color: #888; font-size: 11px;")
        info.setWordWrap(True)
        layout.addWidget(info)

        joints_box = QGroupBox("Zglobovi (radijani)")
        jl = QGridLayout(joints_box)

        self.dk_sliders = []
        self.dk_inputs  = []
        self.dk_labels  = []

        for i, (name, lo, hi, default) in enumerate([
            ("q1 (rotacija baze)", -3.14, 3.14, 0.0),
            ("q2 (rameni zglob)",  -1.57, 1.57, 0.5),
            ("q3 (laktni zglob)",  -1.57, 1.57, -0.5),
        ]):
            lbl = QLabel(name)
            lbl.setStyleSheet("color: #ccc;")
            jl.addWidget(lbl, i, 0)

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

            # Slider → input i label
            idx = i
            def on_slider(val, ii=idx):
                rad = val / 100.0
                self.dk_inputs[ii].setText(f"{rad:.3f}")
                self.dk_labels[ii].setText(f"{rad:.3f} rad")
                self._update_dk_position()
            slider.valueChanged.connect(on_slider)

            # Input → slider i label
            def on_input(text, ii=idx):
                try:
                    rad = float(text)
                    self.dk_sliders[ii].blockSignals(True)
                    self.dk_sliders[ii].setValue(int(rad * 100))
                    self.dk_sliders[ii].blockSignals(False)
                    self.dk_labels[ii].setText(f"{rad:.3f} rad")
                    self._update_dk_position()
                except ValueError:
                    pass
            val_input.textChanged.connect(on_input)

            self.dk_sliders.append(slider)
            self.dk_inputs.append(val_input)
            self.dk_labels.append(val_lbl)

        layout.addWidget(joints_box)

        # Pozicija end-effektora
        pos_box = QGroupBox("Pozicija end-effektora (direktna kinematika)")
        pl = QHBoxLayout(pos_box)
        self.dk_x_lbl = QLabel("x = —")
        self.dk_y_lbl = QLabel("y = —")
        self.dk_z_lbl = QLabel("z = —")
        for l in [self.dk_x_lbl, self.dk_y_lbl, self.dk_z_lbl]:
            l.setStyleSheet("color: #4fc3f7; font-size: 13px;")
            l.setAlignment(Qt.AlignCenter)
            pl.addWidget(l)
        layout.addWidget(pos_box)

        self._update_dk_position()

        # Gumb pošalji
        btn = QPushButton("▶  Pošalji na robot")
        btn.setObjectName("send_btn")
        btn.clicked.connect(self._send_dk)
        layout.addWidget(btn)
        layout.addStretch()
        return w

    def _update_dk_position(self):
        try:
            q1 = float(self.dk_inputs[0].text())
            q2 = float(self.dk_inputs[1].text())
            q3 = float(self.dk_inputs[2].text())
            x, y, z = direct_kinematics(q1, q2, q3)
            self.dk_x_lbl.setText(f"x = {x:.4f} m")
            self.dk_y_lbl.setText(f"y = {y:.4f} m")
            self.dk_z_lbl.setText(f"z = {z:.4f} m")
        except Exception:
            pass

    def _send_dk(self):
        try:
            q1 = float(self.dk_inputs[0].text())
            q2 = float(self.dk_inputs[1].text())
            q3 = float(self.dk_inputs[2].text())
            self.robot_node.send_joints(q1, q2, q3)
            self._set_status(f"✓ Poslano: q1={q1:.3f}, q2={q2:.3f}, q3={q3:.3f}")
        except Exception as e:
            self._set_status(f"✗ Greška: {e}", error=True)

    # ── TAB 2: Inverzna kinematika ──────────────────────────────────────
    def _build_manual_ik_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setSpacing(10)

        info = QLabel("Unesi željenu poziciju end-effektora → robot se pomiče → prikazuju se kutovi zglobova.")
        info.setStyleSheet("color: #888; font-size: 11px;")
        info.setWordWrap(True)
        layout.addWidget(info)

        pos_box = QGroupBox("Pozicija end-effektora (metri)")
        pl = QGridLayout(pos_box)

        self.ik_inputs = {}
        for i, (axis, default) in enumerate([("x", 0.15), ("y", 0.10), ("z", 0.0)]):
            pl.addWidget(QLabel(f"{axis} (m):"), i, 0)
            inp = QLineEdit(str(default))
            inp.textChanged.connect(self._update_ik_joints)
            pl.addWidget(inp, i, 1)
            self.ik_inputs[axis] = inp

        layout.addWidget(pos_box)

        # Zglobovi
        joints_box = QGroupBox("Kutovi zglobova (inverzna kinematika)")
        jl = QHBoxLayout(joints_box)
        self.ik_q1_lbl = QLabel("q1 = —")
        self.ik_q2_lbl = QLabel("q2 = —")
        self.ik_q3_lbl = QLabel("q3 = —")
        for l in [self.ik_q1_lbl, self.ik_q2_lbl, self.ik_q3_lbl]:
            l.setStyleSheet("color: #4fc3f7; font-size: 13px;")
            l.setAlignment(Qt.AlignCenter)
            jl.addWidget(l)
        layout.addWidget(joints_box)

        self._update_ik_joints()

        btn = QPushButton("▶  Pošalji na robot")
        btn.setObjectName("send_btn")
        btn.clicked.connect(self._send_ik)
        layout.addWidget(btn)
        layout.addStretch()
        return w

    def _update_ik_joints(self):
        try:
            x = float(self.ik_inputs["x"].text())
            y = float(self.ik_inputs["y"].text())
            z = float(self.ik_inputs["z"].text())
            q1, q2, q3 = inverse_kinematics(x, y, z)
            self.ik_q1_lbl.setText(f"q1 = {q1:.4f} rad")
            self.ik_q2_lbl.setText(f"q2 = {q2:.4f} rad")
            self.ik_q3_lbl.setText(f"q3 = {q3:.4f} rad")
            self._last_ik = (q1, q2, q3)
        except ValueError as e:
            self.ik_q1_lbl.setText("—")
            self.ik_q2_lbl.setText("izvan dosega")
            self.ik_q3_lbl.setText("—")
            self._last_ik = None
        except Exception:
            self._last_ik = None

    def _send_ik(self):
        self._update_ik_joints()
        if hasattr(self, '_last_ik') and self._last_ik:
            q1, q2, q3 = self._last_ik
            self.robot_node.send_joints(q1, q2, q3)
            x = float(self.ik_inputs["x"].text())
            y = float(self.ik_inputs["y"].text())
            z = float(self.ik_inputs["z"].text())
            self._set_status(f"✓ Poslano na poziciju: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        else:
            self._set_status("✗ Pozicija izvan dosega robota!", error=True)

    # ── TAB 3: Autonomni mod ────────────────────────────────────────────
    def _build_autonomous_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setSpacing(10)

        info = QLabel(
            "Unesi prirodnojezičnu naredbu. Primjer: 'spoji avion i auto, izbjegni loptu'.\n"
            "Sustav će prepoznati objekte i planirati putanju."
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

        log_box = QGroupBox("Log")
        ll = QVBoxLayout(log_box)
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setMinimumHeight(200)
        self.log_output.setPlaceholderText("Ovdje će se prikazivati log izvršavanja...")
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

        self.log_output.append(f">> {cmd}")

        # Parsiranje naredbe - traži 'spoji X i Y, izbjegni Z'
        cmd_lower = cmd.lower()
        start_obj, target_obj, avoid_obj = None, None, None

        try:
            if "spoji" in cmd_lower:
                after_spoji = cmd_lower.split("spoji")[1]
                if " i " in after_spoji:
                    parts = after_spoji.split(" i ")
                    start_obj = parts[0].strip().split(",")[0].strip()
                    rest = parts[1]
                    if "izbjegni" in rest:
                        target_obj = rest.split("izbjegni")[0].strip().rstrip(",").strip()
                        avoid_obj = rest.split("izbjegni")[1].strip()
                    else:
                        target_obj = rest.strip()
            elif "connect" in cmd_lower:
                after = cmd_lower.split("connect")[1]
                if " and " in after:
                    parts = after.split(" and ")
                    start_obj = parts[0].strip()
                    rest = parts[1]
                    if "avoid" in rest:
                        target_obj = rest.split("avoid")[0].strip().rstrip(",").strip()
                        avoid_obj = rest.split("avoid")[1].strip()
                    else:
                        target_obj = rest.strip()
        except Exception:
            pass

        if start_obj and target_obj:
            self.log_output.append(f"   Start objekt: {start_obj}")
            self.log_output.append(f"   Ciljni objekt: {target_obj}")
            if avoid_obj:
                self.log_output.append(f"   Izbjegavaj: {avoid_obj}")
            self.log_output.append("   Pokretanje YOLO detekcije i planiranja putanje...")
            self._set_status(f"▶ Izvršavam: {start_obj} → {target_obj}")
            # TODO: ovdje pozovi autonomni node s start_obj i target_obj
        else:
            self.log_output.append("   ✗ Nisam prepoznao objekte. Koristi format: 'spoji X i Y, izbjegni Z'")
            self._set_status("✗ Format naredbe nije prepoznat.", error=True)

    # ── Helpers ─────────────────────────────────────────────────────────
    def _set_status(self, msg, error=False):
        color = "#ef5350" if error else "#4fc3f7"
        self.status_label.setStyleSheet(f"color: {color}; font-size: 11px;")
        self.status_label.setText(msg)

# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────
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
