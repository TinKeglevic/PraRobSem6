import rclpy
from rclpy.node import Node
import math
import logging
import os
from datetime import datetime
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RRRKinematics:
    """FK i IK za 3-DOF planarni robot."""

    def __init__(self, L1, L2, L3):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def calculate_joint_angles(self, x, y, z):
        """IK — iz željene XYZ pozicije vraca kutove zglobova (radijani)."""
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        z_proj = z - self.L1
        D = math.sqrt(r**2 + z_proj**2)

        if D > (self.L2 + self.L3):
            raise ValueError(
                f"Cilj predaleko: D={D:.3f} m, maksimum={self.L2 + self.L3:.3f} m"
            )
        if D < abs(self.L2 - self.L3):
            raise ValueError(f"Cilj preblizu bazi: D={D:.3f} m")

        cos_theta3 = (D**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta3 = max(min(cos_theta3, 1.0), -1.0)
        theta3 = math.acos(cos_theta3)

        alpha = math.atan2(r, z_proj)
        beta = math.atan2(
            self.L3 * math.sin(theta3),
            self.L2 + self.L3 * math.cos(theta3)
        )
        theta2 = alpha - beta

        return theta1, theta2, theta3

    def compute_fk(self, q1, q2, q3):
        """FK — iz kutova zglobova vraca XYZ poziciju vrha markera."""
        r_link = self.L2 * math.sin(q2) + self.L3 * math.sin(q2 + q3)
        z = self.L1 + self.L2 * math.cos(q2) + self.L3 * math.cos(q2 + q3)
        x = r_link * math.cos(q1)
        y = r_link * math.sin(q1)
        return x, y, z


class KinematicsNode(Node):
    def __init__(self):
        super().__init__("kinematics_node")

        self._setup_file_logger()

        self.kinematics = RRRKinematics(
            L1=0.084,
            L2=0.200,
            L3=0.167,
        )

        self.current_angles = [0.0, 0.0, 0.0]
        self.commanded_angles = [0.0, 0.0, 0.0]

        # Subscriberi
        self.create_subscription(
            Point, "/marker_target", self.on_target_received, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_states_received, 10
        )

        # Publisheri
       
        self.cmd_publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.target_state_publisher = self.create_publisher(
            JointState, "/rrr_arm/target_joint_states", 10
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

    def on_joint_states_received(self, msg):
        if len(msg.position) >= 3:
            self.current_angles = list(msg.position)
            self._log_tracking_error()

    def on_target_received(self, msg):
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
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["zglob_1", "zglob_2", "zglob_3"]
        point = JointTrajectoryPoint()
        point.positions = [float(q1), float(q2), float(q3)]
        msg.points.append(point)
        self.cmd_publisher.publish(msg)

    def _publish_target_joint_states(self, q1, q2, q3):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["zglob_1", "zglob_2", "zglob_3"]
        msg.position = [float(q1), float(q2), float(q3)]
        self.target_state_publisher.publish(msg)

    def _log_tracking_error(self):
        joint_names = ["zglob_1", "zglob_2", "zglob_3"]
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
        self.get_logger().info(log_str)
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