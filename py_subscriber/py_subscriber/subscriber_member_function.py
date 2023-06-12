import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2

class MinimalSubscriber(Node):
    def __init__(self):
        print("Subscriber runned")
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()


    def listener_callback(self, msg):
        print("Got image")
        current_frame = self.br.imgmsg_to_cv2(msg)
        #self.get_logger().info(current_frame.shape)
        print(current_frame)
        cv2.imwrite("recv.png", current_frame)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
