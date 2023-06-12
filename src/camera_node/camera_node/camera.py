import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_general')
        self.general_camera_publisher = self.create_publisher(Image, 'camera_general', 10)
        self.field_camera_publisher = self.create_publisher(Image, 'camera_field', 10)

        # OpenCV bridge setup 
        self.br = CvBridge()
        self.real_image = False
        
        self.cap = cv2.VideoCapture(0)

        # self.send_image = cv2.imread("send_me.png")
        self.general_image = np.zeros((720, 1280, 3), dtype=np.int8)
        self.general_image = cv2.putText(self.general_image, 'General Camera View (Showed when camera disabled in publisher)', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 255, 255), 2, cv2.LINE_AA)
        
        self.field_image = np.zeros((720, 1280, 3), dtype=np.int8)
        self.field_image = cv2.putText(self.field_image, 'Field Camera View', (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 
                   5, (255, 255, 255), 2, cv2.LINE_AA)
        while True:
            # We can read images here
            # status, self.general_image = self.cap.read()
            self.general_camera_publisher.publish(self.br.cv2_to_imgmsg(self.general_image))
            self.field_camera_publisher.publish(self.br.cv2_to_imgmsg(self.field_image))
                   

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
