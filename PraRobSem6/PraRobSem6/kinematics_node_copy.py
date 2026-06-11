import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math
import logging
import os
from datetime import datetime
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


class RRRKinematics:

    def __init__(self, L1, L2, L3):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def calculate_joint_angles(self, x, y, z):
        """IK — iz željene XYZ pozicije vraca kutove zglobova (radijani)."""

        x = max(0.00000001, x)
        y = max(0.00000001, y)
        z = max(0.00000001, z)

        r = distance(x, y, 0, 0)
        l = distance(r, z, 0, self.L1)

        q1 = math.atan(y / x)
        if y < 0:
            q1 += math.pi

        q2 = math.acos((self.L1 - z) / l)
        q3 = 0.5*math.pi
        print(q2, q3)

        if l < self.L2 + self.L3:
            q2 += math.acos((l**2 + self.L2**2 - self.L3**2) / (2*l*self.L2))
            q3 -= math.acos((l**2 + self.L3**2 - self.L2**2) / (2*l*self.L3))
            print(q2, q3)

        q2 = 0.5*math.pi - q2

        return q1, q2, q3

    def compute_fk(self, q1, q2, q3):
        """FK — iz kutova zglobova vraca XYZ poziciju vrha markera."""
        r_link = self.L2 * math.sin(q2) + self.L3 * math.sin(q2 + q3)
        z = self.L1 + self.L2 * math.cos(q2) + self.L3 * math.cos(q2 + q3)
        x = r_link * math.cos(q1)
        y = r_link * math.sin(q1)
        return x, y, z

    def test(rrrk, x, y, z):
        q1, q2, q3 = rrrk.calculate_joint_angles(x, y, z)
        x1, y1, z1 = rrrk.compute_fk(q1, q2, q3)
        print("kutevi", (q1, q2, q3))
        print("tocka", (x1, y1, z1))


class KinematicsNode(Node):
    def __init__(self):
        super().__init__("kinematics_node")

        self._setup_file_logger()

        self.kinematics = RRRKinematics(
            L1=0.088,
            L2=0.200,
            L3=0.19,
        )

        self.joint_names = [
            "joint1",
            "joint2",
            "joint3"
        ]

        self.current_angles = [0.0, 0.0, 0.0]
        self.commanded_angles = [0.0, 0.0, 0.0]

        # Subscriberi
        self.create_subscription(
            Point, "/marker_target", self.marker_target_callback, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        # Publisheri
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.target_state_publisher = self.create_publisher(
            JointState, "/joint_trajectory_controller/target_joint_states", 10
        )

        self.get_logger().info("Kinematics node pokrenut.")
        self.file_logger.info("Kinematics node pokrenut.")

    def _setup_file_logger(self):
        log_dir = os.path.expanduser("~/kinematics_logs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(log_dir, f"session_{timestamp}.log")

        self.file_logger = logging.getLogger("rrr_kinematics")
        self.file_logger.setLevel(logging.DEBUG)
        handler = logging.FileHandler(log_path)
        handler.setFormatter(
            logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
        )
        self.file_logger.addHandler(handler)
        self.get_logger().info(f"Log datoteka: {log_path}")

    def joint_states_callback(self, msg):
        if len(msg.position) >= 3:
            self.current_angles = list(msg.position)
            self._log_tracking_error()

    def marker_target_callback(self, msg):
        print("yeet abc 123")
        x, y, z = msg.x, msg.y, msg.z
        try:
            q1, q2, q3 = self.kinematics.calculate_joint_angles(x, y, z)
            self.commanded_angles = [q1, q2, q3]

            self.file_logger.info(
                f"Cilj ({x:.3f}, {y:.3f}, {z:.3f}) m -> "
                f"q1={math.degrees(q1):.1f}°, "
                f"q2={math.degrees(q2):.1f}°, "
                f"q3={math.degrees(q3):.1f}°"
            )

            self.get_logger().info(  # ← dodaj ovo
            f"Cilj ({x:.3f}, {y:.3f}, {z:.3f}) m -> "
            f"q1={math.degrees(q1):.1f}°, "
            f"q2={math.degrees(q2):.1f}°, "
            f"q3={math.degrees(q3):.1f}°"
            )

            self._send_joint_command(q1, q2, q3)
            self._publish_target_joint_states(q1, q2, q3)

        except ValueError as e:
            self.get_logger().error(f"IK greška: {e}")
            self.file_logger.error(
                f"IK greška za cilj ({x:.3f}, {y:.3f}, {z:.3f}): {e}"
            )

    def _send_joint_command(self, q1, q2, q3):

        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        for joint_name in self.joint_names:
            joint_trajectory_msg.joint_names.append(joint_name)
        
        goal_point = JointTrajectoryPoint()
        goal_point.positions.append(float(q1))
        goal_point.positions.append(float(q2))
        goal_point.positions.append(float(q3))
        goal_point.time_from_start = Duration(seconds=2.0).to_msg()

        joint_trajectory_msg.points.append(goal_point)
        self.joint_trajectory_publisher.publish(joint_trajectory_msg)

    def _publish_target_joint_states(self, q1, q2, q3):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names[:]
        msg.position = [float(q1), float(q2), float(q3)]
        self.target_state_publisher.publish(msg)

    def _log_tracking_error(self):
        joint_names = self.joint_names[:]
        lines = []
        for name, cmd, cur in zip(
            joint_names, self.commanded_angles, self.current_angles
        ):
            err = math.degrees(cmd - cur)
            lines.append(
                f"{name}: naredeno={math.degrees(cmd):.1f}°, "
                f"stvarno={math.degrees(cur):.1f}°, "
                f"greška={err:.2f}°"
            )
        log_str = " | ".join(lines)
        # self.get_logger().info(log_str)
        self.file_logger.debug(f"Tracking: {log_str}")


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()