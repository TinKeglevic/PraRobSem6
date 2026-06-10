#!/usr/bin/env python3
"""
Autonomous node — spaja NLP naredbu, YOLO detekcije i path planner.

Flow:
  /nlp_command (String)  →  parsira "spoji X i Y, izbjegni Z"
  /yolo/detections       →  dohvati bbox centar za svaki objekt
  image2world            →  pretvori pixel → robot koordinate (metri)
  /path_request (String) →  JSON na path_planning_node

Publishea:
  /path_request  (std_msgs/String)
  /detected_objects (std_msgs/String)  — za GUI prikaz

Pretpostavka: T_cam_world.npy postoji u ~/ros2_ws/
Ako ne postoji, treba jednom pokrenuti camera_to_world.py s checkerboardom
i pohraniti matricu (uputa u README).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo

from yolo_msgs.msg import DetectionArray

import json
import os
import numpy as np
import math

# image2world iz prarob_calib
from prarob_calib.camera_to_world import image2world


# ─────────────────────────────────────────────
#  Jednostavni NLP parser
# ─────────────────────────────────────────────

def parse_nlp_command(cmd: str):
    """
    Parsira naredbu poput:
      "spoji avion i auto, izbjegni loptu"
      "connect plane and car, avoid ball"
    Vraca: (connect_objects: list[str], avoid_objects: list[str])
    """
    cmd = cmd.lower().strip()

    connect = []
    avoid   = []

    # Pokušaj razdvojiti na dio za spajanje i dio za izbjegavanje
    # Podržava: "spoji/connect ... i/and ..., izbjegni/avoid ..."
    import re

    # Izbjegavanje
    avoid_match = re.search(
        r'(?:izbjegni|avoid|preskoci|preskoči|ne diraj|skip)\s+(.+?)(?:$|,|\.|;)', cmd
    )
    if avoid_match:
        avoid_str = avoid_match.group(1)
        avoid = [o.strip() for o in re.split(r'\s*(?:i|and|,)\s*', avoid_str) if o.strip()]

    # Spajanje
    connect_match = re.search(
        r'(?:spoji|connect|poveži|povezi)\s+(.+?)(?:izbjegni|avoid|ne diraj|skip|$|;)', cmd
    )
    if connect_match:
        connect_str = connect_match.group(1).strip().rstrip(',').strip()
        connect = [o.strip() for o in re.split(r'\s*(?:i|and|,)\s*', connect_str) if o.strip()]

    return connect, avoid


def best_match(name: str, detections: dict) -> str | None:
    """Nađi najbliži ključ u detections rječniku (fuzzy match)."""
    name = name.lower()
    # Točno podudaranje
    for key in detections:
        if name == key.lower():
            return key
    # Parcijalno podudaranje
    for key in detections:
        if name in key.lower() or key.lower() in name:
            return key
    return None


# ─────────────────────────────────────────────
#  ROS2 node
# ─────────────────────────────────────────────

class AutonomousNode(Node):

    T_CAM_WORLD_PATH = os.path.expanduser('~/ros2_ws/src/PraRobSem6/T_cam_world.npy')
    Z_DRAW = 0.0  # visina crtanja u robot koordinatama

    def __init__(self):
        super().__init__('autonomous_node')

        # Učitaj T_cam_world matricu
        self.T_cam_world = None
        self.k_matrix    = None
        self._load_T_cam_world()

        # Pohrani zadnje detekcije: {class_name: (pixel_x, pixel_y)}
        self.last_detections: dict[str, tuple] = {}

        # Subscriberi
        self.create_subscription(String, '/nlp_command', self._on_nlp, 10)
        self.create_subscription(DetectionArray, '/yolo/detections', self._on_yolo, 10)
        self.create_subscription(CameraInfo, '/camera1/camera_info', self._on_camera_info, 10)

        # Publisheri
        self.path_req_pub   = self.create_publisher(String, '/path_request', 10)
        self.detected_pub   = self.create_publisher(String, '/detected_objects', 10)

        self.get_logger().info('AutonomousNode spreman.')
        if self.T_cam_world is None:
            self.get_logger().warn(
                f'T_cam_world.npy nije pronađen na {self.T_CAM_WORLD_PATH}. '
                'Pokreni camera_to_world node s checkerboardom, zatim spremi matricu.'
            )

    def _load_T_cam_world(self):
        if os.path.exists(self.T_CAM_WORLD_PATH):
            self.T_cam_world = np.load(self.T_CAM_WORLD_PATH)
            self.get_logger().info(f'Učitana T_cam_world iz {self.T_CAM_WORLD_PATH}')
        else:
            self.get_logger().warn(f'Nema {self.T_CAM_WORLD_PATH}')

    def _on_camera_info(self, msg):
        if self.k_matrix is None:
            self.k_matrix = np.reshape(np.array(msg.k), (3, 3))
            self.get_logger().info('K matrica učitana.')
            self.destroy_subscription(
                [s for s in self.subscriptions if s.topic_name == '/camera1/camera_info'][0]
            )

    def _on_yolo(self, msg):
        detections = {}
        for det in msg.detections:
            name = det.class_name.lower()
            cx   = det.bbox.center.position.x
            cy   = det.bbox.center.position.y
            # Ako ima više istih klasa, uzmi prvu (ili po potrebi sve)
            if name not in detections:
                detections[name] = (cx, cy)

        self.last_detections = detections

        # Publishaj popis za GUI
        names_str = ', '.join(detections.keys())
        msg_out = String()
        msg_out.data = names_str
        self.detected_pub.publish(msg_out)

    def _on_nlp(self, msg):
        cmd = msg.data.strip()
        self.get_logger().info(f'NLP naredba: "{cmd}"')

        connect_names, avoid_names = parse_nlp_command(cmd)
        self.get_logger().info(f'  Spoji: {connect_names}')
        self.get_logger().info(f'  Izbjegni: {avoid_names}')

        if len(connect_names) < 2:
            self.get_logger().error('Trebam barem 2 objekta za spajanje.')
            return

        if not self.last_detections:
            self.get_logger().error('Nema YOLO detekcija. Je li YOLO pokrenut?')
            return

        if self.T_cam_world is None:
            self.get_logger().error('T_cam_world nije učitan. Kalibracija kamere nedostaje.')
            return

        if self.k_matrix is None:
            self.get_logger().error('K matrica nije učitana. Čeka /camera1/camera_info.')
            return

        # Pronađi objekte u detekcijama
        key_a = best_match(connect_names[0], self.last_detections)
        key_b = best_match(connect_names[1], self.last_detections)

        if key_a is None:
            self.get_logger().error(f'Objekt "{connect_names[0]}" nije detektiran.')
            return
        if key_b is None:
            self.get_logger().error(f'Objekt "{connect_names[1]}" nije detektiran.')
            return

        # Pretvori pixel → robot koordinate
        start_world = self._pixel_to_robot(self.last_detections[key_a])
        goal_world  = self._pixel_to_robot(self.last_detections[key_b])

        if start_world is None or goal_world is None:
            self.get_logger().error('image2world greška.')
            return

        # Prepreke
        obstacles_world = []
        for avoid_name in avoid_names:
            key_obs = best_match(avoid_name, self.last_detections)
            if key_obs:
                obs_w = self._pixel_to_robot(self.last_detections[key_obs])
                if obs_w is not None:
                    obstacles_world.append([float(obs_w[0]), float(obs_w[1])])
                    self.get_logger().info(
                        f'  Prepreka "{key_obs}": x={obs_w[0]:.3f}, y={obs_w[1]:.3f}'
                    )
            else:
                self.get_logger().warn(f'  Prepreka "{avoid_name}" nije detektirana, ignorira se.')

        self.get_logger().info(
            f'Start: x={start_world[0]:.3f}, y={start_world[1]:.3f} | '
            f'Goal:  x={goal_world[0]:.3f}, y={goal_world[1]:.3f}'
        )

        # Pošalji na path_planning_node
        request = {
            'start':     [float(start_world[0]), float(start_world[1])],
            'goal':      [float(goal_world[0]),  float(goal_world[1])],
            'obstacles': obstacles_world,
        }
        req_msg = String()
        req_msg.data = json.dumps(request)
        self.path_req_pub.publish(req_msg)
        self.get_logger().info('Path request poslan.')

    def _pixel_to_robot(self, pixel_xy):
        """Pretvori (pixel_x, pixel_y) → (x, y) u robot koordinatama (metri)."""
        try:
            pt = image2world(
                np.array(pixel_xy),
                self.k_matrix,
                self.T_cam_world,
                z=self.Z_DRAW,
            )
            # image2world vraca (3,1) array — uzmi x i y
            return float(pt[0]), float(pt[1])
        except Exception as e:
            self.get_logger().error(f'image2world greška: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()