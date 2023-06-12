import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from flask import Flask, render_template
import threading
import os

class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask')
        self.subscription = self.create_subscription(
            Image,
            'camera_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.flask_app = Flask(__name__)
        


        @self.flask_app.route("/")
        def index():
            return render_template("index.html")

        threading.Thread(target=lambda: self.flask_app.run(host='0.0.0.0', port=8080)).start()


    def listener_callback(self, msg):
        print("Image")
        current_frame = self.br.imgmsg_to_cv2(msg)
    
    # @self.flask_app.route("/")
    # def index():
    #     return "Flask works!"


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = FlaskNode()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
