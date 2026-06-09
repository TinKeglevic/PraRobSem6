import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo, Image
from yolo_msgs.msg import DetectionArray  # Updated import

class YoloProcessingNode(Node):
    def __init__(self):
        super().__init__('yolo_processing_node')
        
        self.bridge = CvBridge()
        self.image = None
        

        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('yolo_detections_topic', '/yolo/detections')
        self.declare_parameter('output_topic', 'detection_boxes')

        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        yolo_topic = self.get_parameter('yolo_detections_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value


        self.info_sub = self.create_subscription(
            CameraInfo,
            info_topic,
            self.camera_info_callback,
            10
        )

        self.yolo_sub = self.create_subscription(
            DetectionArray,
            yolo_topic,
            self.yolo_callback,
            10
        )


        self.image_pub = self.create_publisher(Image, out_topic, 10)
        
    def camera_info_callback(self, msg):

        self.image = np.zeros((msg.height, msg.width, 3), dtype=np.uint8)
        self.get_logger().info(f"Empty image created: {msg.width}x{msg.height}")
        
        # stop this subscriber immediately
        self.destroy_subscription(self.info_sub)

    def yolo_callback(self, msg):
        if self.image is None:
            self.get_logger().warn(f"Skipping")
            return
        # clean image for output
        out_image = self.image.copy()

        for i, detection in enumerate(msg.detections):


            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            w  = detection.bbox.size.x
            h  = detection.bbox.size.y


            # OpenCV rectangle coordinates
            start_point = (int(cx - w / 2), int(cy - h / 2))
            end_point   = (int(cx + w / 2), int(cy + h / 2))

            # BGR format for OpenCV
            color = (
                int((i * 75) % 255), 
                int((i * 150) % 255), 
                int((i * 225) % 255)
            )

            cv2.rectangle(out_image, start_point, end_point, color, thickness=3)
            
            # Optional: Add class label if your yolo_msgs supports it
            cv2.putText(out_image, f"ID: {detection.class_name}", (start_point[0], start_point[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # publish
        output_msg = self.bridge.cv2_to_imgmsg(out_image, encoding="bgr8")
        self.image_pub.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()