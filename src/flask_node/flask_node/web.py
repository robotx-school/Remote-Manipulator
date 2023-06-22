import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from flask import Flask, render_template, Response, request, jsonify, abort
import threading
import cv2
import time
import json


class Config:
    def __init__(self, port: int, host: str, joystick_offset: float) -> None:
        self.port = port
        self.host = host
        self.joystick_offset = joystick_offset

class FlaskConnectionNodes(Node):
    def __init__(self, state):
        super().__init__('flask')


        # Declare node params with defaults
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', "0.0.0.0")
        self.declare_parameter('joystick_offset', 0.01)

        self.flask_config = Config(self.get_parameter('port').value, self.get_parameter('host').value, self.get_parameter('joystick_offset').value)


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
        
        self.urx_status_subcriber = self.create_subscription(
            String,
            'urx_status',
            self.urx_status_callback,
            1
        )

        self.robot_control = self.create_publisher(
            String,
            'robot_control', # FIXIT; use robot control node here
            1
        )

        self.general_camera_sub
        self.field_camera_sub
        self.state = state
        
    def general_camera_callback(self, image):
        self.state.general_camera_image = self.br.imgmsg_to_cv2(image)

    def field_camera_callback(self, image):
        self.state.field_camera_image = self.br.imgmsg_to_cv2(image)

    def urx_status_callback(self, data):
        data = json.loads(data.data)
        self.state.urx_status["position"] = data["position"]
        self.state.urx_status["mode"] = data["mode"]
        self.state.urx_status["ip"] = data["ip"]
        self.state.urx_status["connected"] = data["connected"]
        self.state.urx_status["gripper"] = data["gripper"]
        # self.get_logger().info(f"{self.state.urx_status}")

class StateManager:
    def __init__(self) -> None:
        self.general_camera_image = None
        self.field_camera_image = None
        self.urx_status = {
            "position": [-2] * 6,
            "mode": "None",
            "ip": "None",
            "connected": "None",
            "gripper": -2
        }

class FlaskApp:
    def __init__(self, state, urx_command_publish, joystick_offset) -> None:
        self.app = Flask(__name__)
        self.state = state
        self.urx_command_publish = urx_command_publish

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
            movel = json.loads(request.json["movel"])
            if self.state.urx_status["connected"]:
                send_data = String()
                send_data.data = json.dumps({
                    "type": "movel",
                    "data": movel
                })
                self.urx_command_publish.publish(send_data)
                # self.robot.robot_conn.movel(content, acc=0.2, vel=0.2, wait=False)
                return jsonify({"status": True})
            else:
                return abort(401, "Robot disconnected")
        @self.app.route("/api/gripper", methods=['POST'])
        def api_gripper():
            pose = request.json["pose"]
            if self.state.urx_status["connected"]:
                send_data = String()
                send_data.data = json.dumps({
                    "type": "gripper",
                    "data": pose
                })
                self.urx_command_publish.publish(send_data)
                return jsonify({"status": True})
            else:
                return abort(401, "Robot disconnected")

        @self.app.route("/api/gripper/set", methods=['POST'])
        def api_set_gripper():
            pass

        @self.app.route("/api/get_data")
        def api_getl():
            response = {"getl": [-1] * 6, "gripper": "N/A", "mode": "N/A", "ip": "N/A", "connected": self.state.urx_status["connected"]}
            if self.state.urx_status["connected"]:
                curr_l = self.state.urx_status["position"]
                curr_l = list(map(lambda x: round(x, 3), curr_l))
                response["getl"] = curr_l
                response["mode"] = self.state.urx_status["mode"]
                response["ip"] = self.state.urx_status["ip"]
                response["gripper"] = self.state.urx_status["gripper"]

            return response
        
        @self.app.route("/api/system", methods=['POST'])
        def api_system_commands():
            if not self.state.urx_status["connected"]:
                return abort(401, "Robot disconnected")
            
            dt = request.json
            command = dt["command"]
            extra = dt["extra"] if "extra" in dt else ""
            send_data = String()
            if command in ["power_on", "power_off", "shutdown", "brake_release", "close_popup", "show_popup"]:
                send_data.data = json.dumps({
                    "type": "dashboard",
                    "data": command,
                    "extra": extra
                })
                self.urx_command_publish.publish(send_data)
            else:
                return abort(400, "Incorrect system command")
            
            return jsonify({"status": True})
        
        @self.app.route("/api/robot/set_ip", methods=['POST'])
        def api_robot_set_ip():
            new_ip = request.json["ip"]
            send_data = String()
            send_data.data = json.dumps({
                "type": "change_ip",
                "ip": new_ip
            })
            self.urx_command_publish.publish(send_data)
            return jsonify({"status": True, "detail": new_ip})

        @self.app.route("/api/joystick", methods=['POST'])
        def joystick_handler():
            dir_ = request.json["dir"]
            # self.robot.logger.info(f"Direction is: {dir_}")
            dir_mapping = {
                    "y-": (0, joystick_offset),
                    "y+": (0, -joystick_offset),
                    "x-": (1, joystick_offset),
                    "x+": (1, -joystick_offset),
                    "z+": (2, joystick_offset),
                    "z-": (2, -joystick_offset)
            }
            if dir_ in list(dir_mapping.keys()):
                if self.state.urx_status["connected"]:
                    new_pos = self.state.urx_status["position"] # get current pose
                    change = dir_mapping[dir_]
                    new_pos[change[0]] += change[1]

                    send_data = String()
                    send_data.data = json.dumps({
                        "type": "movel",
                        "data": new_pos
                    })
                    self.urx_command_publish.publish(send_data)
                    return jsonify({"status": True, "detail": "Move"})
                else:
                    return abort(401, "Robot disconnected")
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

def main(args=None):
    state = StateManager()
    rclpy.init(args=args)
    flask_topics = FlaskConnectionNodes(state)
    app = FlaskApp(state, flask_topics.robot_control, flask_topics.flask_config.joystick_offset)
    
    threading.Thread(target=lambda: app.app.run(port=flask_topics.flask_config.port, host=flask_topics.flask_config.host)).start()

    rclpy.spin(flask_topics)
    flask_topics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
