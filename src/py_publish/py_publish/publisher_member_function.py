import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Opencv 
        self.br = CvBridge()
        self.send_image = cv2.imread("send_me.png")

    def timer_callback(self):
        '''
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        '''
        print("Image sent")
        self.publisher_.publish(self.br.cv2_to_imgmsg(self.send_image))


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
