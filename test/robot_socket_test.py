import socket
import json
from typing import List

class Gripper:
    def __init__(self, socket) -> None:
        self.socket = socket

    def pose(self) -> int:
        self.socket.sendall(json.dumps({
            "type": "gripper_pose"
        }).encode("utf-8"))
        data = json.loads(self.socket.recv(1024))["pose"]
        return data

    def action(self, pose: int):
        self.socket.sendall(json.dumps({
            "type": "gripper_action",
            "pose": pose
        }).encode("utf-8"))
        

class RobotClient:
    def __init__(self, ip: str, port: int = 6666) -> None:
        self.ip = ip
        self.port = port
        self.client_socket = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.ip, self.port))
        self.gripper = Gripper(self.client_socket)

    def movel(self, pose: List[int]) -> None:
        payload = json.dumps({
            "type": "movel",
            "pose": pose
        }).encode("utf-8")
        self.client_socket.sendall(payload)

    def getl(self) -> List[int]:
        self.client_socket.sendall(json.dumps({
            "type": "getl"
        }).encode("utf-8"))
        data = self.client_socket.recv(1024)
        return json.loads(data)["pose"]


if __name__ == "__main__":
    robot = RobotClient("localhost")
    robot.gripper.action(10)

    # robot.movel()
    # robot.getl()
    # robot.gripper.pose()
    # robot.gripper.action()