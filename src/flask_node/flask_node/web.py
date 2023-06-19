import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from flask import Flask, render_template, Response, request, jsonify, abort
import threading
import cv2
import time
import urx
from subprocess import PIPE, run
from typing import List

class CameraSub(Node):
    def __init__(self, state):
        super().__init__('flask')
        # Init CV and Camera topics readers
        self.br = CvBridge()
        self.general_camera_sub = self.create_subscription(
            Image,
            'camera_general',
            self.general_camera_callback,
            1)
        self.general_camera_sub
        
        self.field_camera_sub = self.create_subscription(
            Image,
            'camera_field',
            self.field_camera_callback,
            1)
        self.general_camera_sub
        self.field_camera_sub
        self.state = state
        
        

    def general_camera_callback(self, image):
        self.state.general_camera_image = self.br.imgmsg_to_cv2(image)

    def field_camera_callback(self, image):
        self.state.field_camera_image = self.br.imgmsg_to_cv2(image)

class StateManager:
    def __init__(self) -> None:
        self.general_camera_image = None
        self.field_camera_image = None

class FlaskApp:
    def __init__(self, state, robot, robot_low_level) -> None:
        self.app = Flask(__name__)
        self.state = state
        self.robot = robot
        self.robot_low_level = robot_low_level

        @self.app.route("/")
        def __index():
            return render_template("index.html")
        
        @self.app.route('/general_camera_feed')
        def general_video_feed():
            return Response(self.gen_img("general"), mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/field_camera_feed')
        def field_camera_feed():
            return Response(self.gen_img("field"), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route("/api/movel", methods=['POST'])
        def api_movel():
            content = eval(request.json["movel"])
            if self.robot.connected:
                self.robot.robot_conn.movel(content, acc=0.2, vel=0.2, wait=False)
                return jsonify({"status": True})
            else:
                return abort(400, "Robot disconnected")

        @self.app.route("/api/get_data")
        def api_getl():
            response = {"getl": [-1] * 6, "mode": "N/A", "ip": "N/A"}
            if self.robot.connected:
                curr_l = self.robot.robot_conn.getl()
                curr_l = list(map(lambda x: round(x, 3), curr_l))
                response["getl"] = curr_l
                response["mode"] = self.robot_low_level.get_robot_mode()
                response["ip"] = self.robot_low_level.ip

            return response
        
        @self.app.route("/api/system", methods=['POST'])
        def api_system_commands():
            command = request.json["command"]
            if command == "power_on":
                self.robot_low_level.power_on()
            elif command == "power_off":
                self.robot_low_level.power_off()
            elif command == "shutdown":
                self.robot_low_level.shutdown()
            elif command == "brake_release":
                self.robot_low_level.brake_release()
            else:
                return abort(400, "Incorrect system command")
            
            return jsonify({"status": True})
        
        @self.app.route("/api/robot/set_ip", methods=['POST'])
        def api_robot_set_ip():
            new_ip = request.json["ip"]
            self.robot.disconnect()
            self.robot.connect(new_ip)
            self.robot_low_level.ip = new_ip
            return jsonify({"status": True, "detail": new_ip})

        @self.app.route("/api/joystick", methods=['POST'])
        def joystick_handler():
            dir_ = request.json["dir"]
            self.robot.logger.info(f"Direction is: {dir_}")
            offset = 0.05
            dir_mapping = {
                    "y-": (0, offset),
                    "y+": (0, -offset),
                    "x-": (1, offset),
                    "x+": (1, -offset),
                    "z+": (2, offset),
                    "z-": (2, -offset)
            }
            if dir_ in list(dir_mapping.keys()):
                if self.robot.connected:
                    new_pos = self.robot.robot_conn.getl() # get current pose
                    change = dir_mapping[dir_]
                    new_pos[change[0]] += change[1]
                    self.robot.robot_conn.movel(new_pos, vel=0.2, acc=0.2)
                    return jsonify({"status": True, "detail": "Move"})
                else:
                    return abort(400, "Robot disconnected")
            else:
                return abort(400, "Invalid direction")
            
    
    def gen_img(self, image_type: str):
        while True:
            try:
                image = self.state.general_camera_image
                if image_type == "field":
                    image = self.state.field_camera_image
                ret, buffer = cv2.imencode('.jpg', image)
                frame = buffer.tobytes()
                time.sleep(0.05)
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except cv2.error:
                print(f"Bad {image_type} image, skipping it")


class Robot:
    def __init__(self, ip: str, logger: object) -> None:
        self.connected = False
        self.logger = logger
        self.connect_max_attempts = 5
        self.ip = None
        self.connect(ip)

    def disconnect(self):
        self.connected = False
        self.ip = None
        self.robot_conn.close()

    def connect(self, ip: str):
        connect_attempt = 0
        while not self.connected and connect_attempt < self.connect_max_attempts:
            try:
                self.robot_conn = urx.Robot(ip)
                self.ip = ip
                self.connected = True
                self.logger.info("Connected to robot")
            except Exception as e:
                connect_attempt += 1
                self.logger.error(f"Can't connect to robot, attempt ({connect_attempt}/{self.connect_max_attempts}); More info: {e}")


class RobotLowLevel:
    def __init__(self, ip: str) -> None:
        self.ip = ip

    def build_command(self, command: str) -> str:
        return f'''echo y | plink root@{self.ip} -pw easybot "{{ echo "{command}"; echo "quit"; }} | nc 127.0.0.1 29999"'''

    def execute_command(self, command: str) -> List[str]:
        result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
        result = result.stdout.strip().split("\n")[1:]
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

def main(args=None):
    state = StateManager()
    rclpy.init(args=args)
    camera_sub = CameraSub(state)
    robot_ip_default = "192.168.2.172"
    robot = Robot(robot_ip_default, camera_sub.get_logger())
    robot_low_level = RobotLowLevel(robot_ip_default)
    app = FlaskApp(state, robot, robot_low_level)
    
    threading.Thread(target=lambda: app.app.run(port=8080, host="0.0.0.0")).start()

    rclpy.spin(camera_sub)
    camera_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
