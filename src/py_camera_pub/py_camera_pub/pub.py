import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from termcolor import colored


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('camera_pub')
        self.publisher_ = self.create_publisher(Image, 'image', 1)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        self.loop()
    

    def loop(self):
        self.get_logger().info(f'Called')
        while True:
            status, frame = self.cap.read()
            #enc_start_time = time.time()
            image_type = self.br.cv2_to_imgmsg(frame)
            #enc_finish_time = time.time()
            #self.get_logger().info(f'Encoding took: {enc_finish_time - enc_start_time} seconds')
            #pub_start_time = time.time()
            self.publisher_.publish(image_type)
            #pub_finish_time = time.time()
            #self.get_logger().info(f'Publishing took: {pub_finish_time - pub_start_time} seconds')
            #all_spent = (pub_finish_time - pub_start_time) + (enc_finish_time - enc_start_time)
            #self.get_logger().info(colored(f'In sum: {all_spent} seconds - {1 / all_spent} FPS', 'green'))
            

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