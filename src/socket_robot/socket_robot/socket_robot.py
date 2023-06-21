import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

class UrxNodePub(Node):
    def __init__(self):
        super().__init__('socket_robot')
        self.robot_control_pub = self.create_publisher(String, 'robot_control', 1)

    def send_command(self, data: str):
        send_data = String()
        send_data.data = data
        self.robot_control_pub.publish(send_data)

class SocketServer:
    def __init__(self, robot_control_pub, host: str = "0.0.0.0", port: int = 6666) -> None:
        self.robot_control_pub = robot_control_pub
        self.host = host
        self.port = port

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
    
    def handle_client(self, client_socket):
        while True:
            try:
                data = client_socket.recv(1024)
                if data:
                    self.robot_control_pub.send_command(data.decode("utf-8"))
            except ConnectionResetError: # If client disconnect
                client_socket.close()
                break

    def clients_joiner(self):
        while True:
            client_socket, addr = self.server_socket.accept()
            threading.Thread(target=lambda: self.handle_client(client_socket)).start()


def main(args=None):
    rclpy.init(args=args)

    robot_control_pub = UrxNodePub()
    socket_server = SocketServer(robot_control_pub)
    threading.Thread(target=socket_server.clients_joiner).start()

    rclpy.spin(robot_control_pub)
    robot_control_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()