import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from flask import Flask, render_template, Response, request, jsonify
import threading
import cv2
import time
import urx

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
    def __init__(self, state, robot) -> None:
        self.app = Flask(__name__)
        self.state = state
        self.robot = robot

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
                return jsonify({"status": False})

        @self.app.route("/api/getl")
        def api_getl():
            if self.robot.connected:
                return self.robot.robot_conn.getl()
            else:
                return jsonify([-1] * 6) # can't get info
        
        @self.app.route("/api/system/resume")
        def api_system_resume():
            pass

        @self.app.route("/api/system/restart")
        def api_system_restart():
            pass

        @self.app.route("/api/system/stop")
        def api_system_stop():
            pass

        @self.app.route("/api/joystick")
        def joystick_handler():
            dir_ = request.json["dir"]
            offset = 0.05
            dir_mapping = {
                    "x+": (0, offset),
                    "x-": (0, -offset),
                    "y+": (1, offset),
                    "y-": (1, -offset),
                    "z+": (2, offset),
                    "z-": (2, -offset)
            }
            if dir_ in list(dir_mapping.keys()):
                if self.robot.connected:
                    new_pos = self.robot.robot_conn.getl() # get current pose
                    change = dir_mapping[dir_]
                    new_pose[change[0]] += change[1]
                    return jsonify({"status": True, "detail": "Move"})
                else:
                    return jsonify({"status": False, "detail": "Robot disconnected"})
            else:
                return jsonify({"status": False, "detail": "Invalid direction"})
            
    
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
        connect_attempt = 0
        while not self.connected and connect_attempt < self.connect_max_attempts:
            try:
                self.robot_conn = urx.Robot(ip)
                self.connected = True
                self.logger.info("Connected to robot")
            except Exception as e:
                connect_attempt += 1
                self.logger.error(f"Can't connect to robot, attempt ({connect_attempt}/{self.connect_max_attempts}); More info: {e}")

def main(args=None):
    state = StateManager()
    rclpy.init(args=args)
    camera_sub = CameraSub(state)
    robot = Robot("192.168.2.65", camera_sub.get_logger())
    app = FlaskApp(state, robot)
    
    threading.Thread(target=lambda: app.app.run(port=8080, host="0.0.0.0")).start()

    rclpy.spin(camera_sub)
    camera_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
