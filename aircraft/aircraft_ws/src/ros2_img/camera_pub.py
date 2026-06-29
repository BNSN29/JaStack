import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class IMX477Publisher(Node):
    def __init__(self):
        super().__init__('imx477_publisher')

        self.publisher_ = self.create_publisher(Image, '/Drone1/camera/image_raw', 10)
        self.bridge = CvBridge()

        # GStreamer pipeline for IMX477 on Jetson
        self.pipeline = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=60/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink"
        )

        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Failed to grab frame")
            exit()

        # Convert OpenCV image (numpy array) to ROS2 Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = IMX477Publisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()