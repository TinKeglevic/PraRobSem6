import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


class RVizMarkerNode(Node):
    def __init__(self):
        super().__init__("rviz_marker_node")

        # Dimenzije robota u metrima — mora biti isto kao u kinematics_node
        self.L1 = 0.084
        self.L2 = 0.2
        self.L3 = 0.19

        # Stvarni kutovi s motora -> crvena sfera
        self.create_subscription(
            JointState, "/joint_states", self.on_actual_joints, 10
        )

        # Ciljni kutovi iz IK noda -> zelena sfera
        self.create_subscription(
            JointState, "/rrr_arm/target_joint_states", self.on_target_joints, 10
        )

        self.marker_publisher = self.create_publisher(
            Marker, "/rrr_arm/markers", 10
        )

        self.get_logger().info("RViz marker node pokrenut.")

    def on_actual_joints(self, msg):
        """Stvarni kutovi s motora — crvena sfera."""
        if len(msg.position) >= 3:
            x, y, z = self._fk(*msg.position[:3])
            self._publish_sphere(x, y, z, marker_id=0, r=1.0, g=0.0, b=0.0)

    def on_target_joints(self, msg):
        """Ciljni kutovi iz IK — zelena sfera."""
        if len(msg.position) >= 3:
            x, y, z = self._fk(*msg.position[:3])
            self._publish_sphere(x, y, z, marker_id=1, r=0.0, g=1.0, b=0.0)

    def _fk(self, q1, q2, q3):
        """FK — iz kutova zglobova vraca XYZ poziciju vrha markera."""
        r_link = self.L2 * math.sin(q2) + self.L3 * math.sin(q2 + q3)
        z = self.L1 + self.L2 * math.cos(q2) + self.L3 * math.cos(q2 + q3)
        x = r_link * math.cos(q1)
        y = r_link * math.sin(q1)
        return x, y, z

    def _publish_sphere(self, x, y, z, marker_id, r, g, b):
        """Publishaj sferu u RVIZ na zadanoj poziciji."""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rrr_arm"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.025
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RVizMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()