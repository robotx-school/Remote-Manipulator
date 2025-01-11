import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import List
import urx
from subprocess import PIPE, run
import logging
import threading
import time

from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper



class Robot:
    def __init__(self, gripper_start_pose: int, gripper_step: int) -> None:
        self.connected = False
        self.connect_max_attempts = 3
        self.ip = None
        self.gripper_pose = None
        self.gripper_step = gripper_step
        self.gripper_start_pose = gripper_start_pose
        # self.connect(ip)

    def disconnect(self):
        self.connected = False
        self.ip = None
        self.robot_conn.close()

    def connect(self, ip: str):
        connect_attempt = 0
        while not self.connected and connect_attempt < self.connect_max_attempts:
            try:
                self.robot_conn = urx.Robot(ip)
                self.robotiqgrip = Robotiq_Two_Finger_Gripper(self.robot_conn)
                self.robotiqgrip.gripper_action(self.gripper_start_pose) # Set gripper to 0
                self.gripper_pose = self.gripper_start_pose
                self.ip = ip
                self.connected = True
                logging.info("Connected to robot")
            except Exception as e:
                connect_attempt += 1
                logging.error(f"Can't connect to robot, attempt ({connect_attempt}/{self.connect_max_attempts}); More info: {e}")

    # Low Level to dashboard server
    def build_command(self, command: str) -> str:
        return f'''echo y | plink root@{self.ip} -pw easybot "{{ echo "{command}"; echo "quit"; }} | nc 127.0.0.1 29999"'''

    def execute_command(self, command: str) -> List[str]:
        #logging.error(f"{command}")
        result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
        #logging.error(f"{result.stdout}")
        result = result.stdout.strip().split("\n")[1:]
        #logging.error(f"{result}")
        return result
    
    def get_robot_mode(self) -> str:
        command = self.build_command("robotmode")
        return self.execute_command(command)[0].replace("Robotmode: ", "")
    
    def power_on(self) -> None:
        self.close_popup()
        return self.execute_command(self.build_command("power on"))
    
    def power_off(self) -> None:
        return self.execute_command(self.build_command("power off"))
    
    def shutdown(self) -> None:
        return self.execute_command(self.build_command("shutdown"))
    
    def brake_release(self) -> None:
        self.close_popup()
        return self.execute_command(self.build_command("brake release"))
    
    def close_popup(self) -> None:
        return self.execute_command(self.build_command("close popup"))
    
    def show_popup(self, text: str) -> None:
        return self.execute_command(self.build_command(f"popup {text}"))


class UrxNode(Node):
    def __init__(self):
        super().__init__('urx')


        # Declare node params with defaults
        self.declare_parameter('ip', "192.168.1.2")
        self.declare_parameter('popup_message', "Манипулятор захвачен RobotX")
        self.declare_parameter('status_publish_rate', 0.5)
        self.declare_parameter('gripper_start_pose', 0)
        self.declare_parameter('gripper_step', 10)
        

        self.status_publisher = self.create_publisher(String, 'urx_status', 1)
        self.command_subscriber = self.create_subscription(
            String,
            'urx_command',
            self.control_urx,
            1)

        #self.timer = self.create_timer(5, self.publish_urx_data)
        self.robot = None
        publish_thread = threading.Thread(target=self.publish_urx_data)
        publish_thread.start()

    def publish_urx_data(self):
        self.get_logger().info("Publish thread started")
        while True:
            if self.robot:
                payload = {"position": [-1] * 6, "gripper": "N/A", "mode": "N/A", "ip": "N/A", "connected": self.robot.connected}
                if self.robot.connected:
                    payload["position"] = self.robot.robot_conn.getl()
                    payload["mode"] = self.robot.get_robot_mode()
                    payload["ip"] = self.robot.ip
                    payload["gripper"] = self.robot.gripper_pose
                else:
                    payload["position"] = [-1, -1, -1, -1, -1, -1]
                    payload["mode"] = "NO DATA"
                    payload["ip"] = self.robot.ip
                    payload["gripper"] = 0
                    # self.get_logger().warning('Robot disconnected; No data received')
                send_data = String()
                send_data.data = json.dumps(payload)
                self.status_publisher.publish(send_data)
                time.sleep(0.5)
                

    def control_urx(self, data):
        #self.get_logger().info("Callback")
        if self.robot.connected:
            #self.get_logger().info("Checked connected")
            command = json.loads(data.data)
            #self.get_logger().info(f"{command} dumped")
            if command["type"] == "movel":
                try:
                    self.robot.robot_conn.movel(command["data"], vel=command["velocity"], acc=command["acceleration"])
                    self.get_logger().info("movel sent")
                except Exception as e:
                    self.get_logger().error(f"Error while movel: {e}")
                #self.get_logger().info(f'MoveL to: {command["data"]}')
            elif command["type"] == "dashboard":

                self.get_logger().info(f'Executing command: {command["data"]} {command}')
                if command["data"] == "power_on":
                    self.robot.power_on()
                elif command["data"] == "power_off":
                    self.robot.power_off()
                elif command["data"] == "shutdown":
                    self.robot.shutdown()
                elif command["data"] == "brake_release":
                    self.robot.brake_release()
                elif command["data"] == "close_popup":
                    self.robot.close_popup()
                elif command["data"] == "show_popup":
                    self.robot.show_popup(command["extra"])

            elif command["type"] == "gripper":
                self.robot.gripper_pose = command["data"]
                self.robot.robotiqgrip.gripper_action(command["data"])
            
            elif command["type"] == "change_ip":
                self.robot.disconnect()
                self.robot.connect(command['ip'])
                self.get_logger().info(f"Changing robot ip to: {command['ip']}")

        else:
            self.get_logger().warning("Robot disconnected; Can't execute command")



def main(args=None):
    rclpy.init(args=args)

    
    urx_node = UrxNode()
    robot = Robot(urx_node.get_parameter("gripper_start_pose").value, urx_node.get_parameter("gripper_step").value)
    robot.connect(urx_node.get_parameter("ip").value)
    
    robot.close_popup()
    robot.show_popup(urx_node.get_parameter("popup_message").value)
    urx_node.robot = robot

    rclpy.spin(urx_node)

    urx_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
