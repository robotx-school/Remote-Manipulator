import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import json


class State:
    def __init__(self) -> None:
        self.pose = None
        self.gripper = None

class UrxNodePub(Node):
    def __init__(self, state):
        super().__init__('socket_robot')
        self.state = state
        self.robot_control_pub = self.create_publisher(String, 'robot_control', 1)
        self.robot_status = self.create_subscription(String, 'urx_status', self.robot_data_callback, 1)

    def robot_data_callback(self, data):
        payload = json.loads(data.data)
        self.state.pose = payload["position"]
        self.state.gripper = payload["gripper"]

    def send_command(self, data: str):
        send_data = String()
        send_data.data = data
        self.robot_control_pub.publish(send_data)
    

class SocketServer:
    def __init__(self, robot_control_pub, state, host: str = "0.0.0.0", port: int = 6666) -> None:
        self.robot_control_pub = robot_control_pub
        self.host = host
        self.port = port
        self.state = state

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
    
    def handle_client(self, client_socket):
        while True:
            try:
                data = client_socket.recv(1024)
                if data:
                    try:
                        data = json.loads(data)
                        if data["type"] == "movel":
                            self.robot_control_pub.send_command(json.dumps({
                                "type": "movel",
                                "data": data["pose"]
                            }))
                        elif data["type"] == "getl":
                            client_socket.sendall(json.dumps({"pose": self.state.pose}).encode("utf-8"))
                    except json.decoder.JSONDecodeError:
                        pass
                    
            except ConnectionResetError: # If client disconnect
                client_socket.close()
                break

    def clients_joiner(self):
        while True:
            client_socket, addr = self.server_socket.accept()
            threading.Thread(target=lambda: self.handle_client(client_socket)).start()


def main(args=None):
    rclpy.init(args=args)

    state = State()
    robot_control_pub = UrxNodePub(state)
    socket_server = SocketServer(robot_control_pub, state)
    threading.Thread(target=socket_server.clients_joiner).start()

    rclpy.spin(robot_control_pub)
    robot_control_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()