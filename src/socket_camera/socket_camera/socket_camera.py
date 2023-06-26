import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import socket
import threading
import time
import pickle
import struct

class GeneralCameraSub(Node):
    def __init__(self, state):
        super().__init__('socket_camera')
        self.br = CvBridge()
        
        self.general_camera_sub = self.create_subscription(
            Image,
            'camera_field',
            self.general_camera_callback,
            1)
        self.state = state

    def general_camera_callback(self, image):
        self.state.general_camera_image = self.br.imgmsg_to_cv2(image)


class StateManager:
    def __init__(self) -> None:
        self.general_camera_image = None


class SocketServer:
    def __init__(self, state, host: str = "0.0.0.0", port: int = 9988) -> None:
        self.state = state
        self.host = host
        self.port = port

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)

    def handle_client(self, client_socket):
        if client_socket:
            last_sent_time = time.time()
            while True:
                if time.time() - last_sent_time > 0.3:
                    data = pickle.dumps(self.state.general_camera_image)
                    message = struct.pack("Q", len(data)) + data
                    try:
                        client_socket.sendall(message)
                        last_sent_time = time.time()
                    except (ConnectionResetError, BrokenPipeError):
                        # print('Client disconnected')
                        client_socket.close()
                        break

    def clients_joiner(self):
        while True:
            client_socket, addr = self.server_socket.accept()
            threading.Thread(target=lambda: self.handle_client(client_socket)).start()


def main(args=None):
    rclpy.init(args=args)
    state = StateManager()
    socket_server = SocketServer(state)
    threading.Thread(target=socket_server.clients_joiner).start()

    
    general_camera_subscriber = GeneralCameraSub(state)

    rclpy.spin(general_camera_subscriber)

    general_camera_subscriber.destroy_node()
    rclpy.shutdown()
    socket_server.server_socket.close()



if __name__ == '__main__':
    main()