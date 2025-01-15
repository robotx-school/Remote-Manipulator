import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2
import socket
import struct
import pickle

class CameraClient:
    def __init__(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port
        self.image = None  # Last image stored here
    
        self.client_socket = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((ip, port))
        self.payload_size = struct.calcsize("Q")
        self.data = b""

    def get_frame(self) -> np.ndarray:
        while len(self.data) < self.payload_size:
            packet = self.client_socket.recv(4 * 1024)
            if not packet:
                break
            self.data += packet
        packed_msg_size = self.data[:self.payload_size]
        self.data = self.data[self.payload_size:]
        msg_size = struct.unpack("Q", packed_msg_size)[0]

        while len(self.data) < msg_size:
            self.data += self.client_socket.recv(4 * 1024)
        frame_data = self.data[:msg_size]
        self.data = self.data[msg_size:]
        frame = pickle.loads(frame_data)
        return frame


    def close(self) -> None:
        self.client_socket.close()

class CameraReceiverNode(Node):

    def __init__(self):
        super().__init__('camera_receiver')

        self.bridge = CvBridge()

        self.field_image_publisher = self.create_publisher(Image, 'test_node/camera/field', 10)
        self.remote_camera = CameraClient("localhost", 9988)

        self.timer = self.create_timer(0.2, self.receive)

    def receive(self):
        frame = self.remote_camera.get_frame()
        self.field_image_publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    image_reader = CameraReceiverNode()
    rclpy.spin(image_reader)

if __name__ == '__main__':
    main()
