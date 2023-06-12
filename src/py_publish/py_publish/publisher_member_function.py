import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('camera_topic')
        self.publisher_ = self.create_publisher(Image, 'camera_topic', 10)

        # Opencv 
        self.br = CvBridge()
        #self.send_image = cv2.imread("send_me.png")
        self.send_image = np.zeros((10, 10, 3), dtype=np.int8)
        self.send_image = cv2.putText(self.send_image, 'General Camera View', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 255, 255), 2, cv2.LINE_AA)
        while True:
            self.send()

    def send(self):
        # Use self.cap.read() here
        start = time.time()
        self.publisher_.publish(self.br.cv2_to_imgmsg(self.send_image))
        print(start - time.time())


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
