import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import threading
import numpy as np
import cv2
import time


class CameraSub(Node):
    def __init__(self):
        super().__init__('flask')
        # Init CV and Camera topics readers
        self.br = CvBridge()
        self.general_camera_sub = self.create_subscription(
            Image,
            'image',
            self.general_camera_callback,
            1)
        self.general_camera_sub
        self.last_image_time = time.time()

    def general_camera_callback(self, image):
        tmp = self.br.imgmsg_to_cv2(image)
        self.get_logger().info(f'FPS: {1 / (time.time() - self.last_image_time)}')
        self.last_image_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    camera_sub = CameraSub()
    rclpy.spin(camera_sub)
    camera_sub.destroy_node()
    rclpy.shutdown()
