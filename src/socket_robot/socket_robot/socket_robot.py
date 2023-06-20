import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import threading

class UrxNodePub(Node):
    def __init__(self):
        super().__init__('socket_robot')
        self.urx_command_publisher = self.create_publisher(String, 'urx_command', 1)

    def send_command(self, data: dict):
        send_data = String()
        send_data.data = json.dumps(data)
        self.urx_command_publisher.publish(send_data)

class SocketServer:
    def __init__(self, urx_command_publisher, host: str = "0.0.0.0", port: int = 6666) -> None:
        self.urx_command_publisher = urx_command_publisher
        self.host = host
        self.port = port

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
    
    def handle_client(self, client_socket):
        while True:
            data = client_socket.recv(1024)
            if data:
                data = json.loads(data)
                if type(data) == list and len(data) == 6:
                    self.urx_command_publisher.send_command({"type": "model", "data": data})
                elif type(data) == str:
                    self.urx_command_publisher.send_command({"type": "dashboard", "data": data})




    def clients_joiner(self):
        while True:
            client_socket, addr = self.server_socket.accept()
            threading.Thread(target=lambda: self.handle_client(client_socket)).start()


    

def main(args=None):
    rclpy.init(args=args)

    urx_command_publisher = UrxNodePub()
    socket_server = SocketServer(urx_command_publisher)
    threading.Thread(target=socket_server.clients_joiner).start()

    rclpy.spin(urx_command_publisher)
    urx_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()