import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from flask import Flask, render_template, Response, request
import threading
import numpy as np
import cv2
import time

class CameraSub(Node):
    def __init__(self, state):
        super().__init__('flask')
        # Init CV and Camera topics readers
        self.br = CvBridge()
        self.general_camera_sub = self.create_subscription(
            Image,
            'camera_general',
            self.general_camera_callback,
            10)
        self.general_camera_sub
        
        self.field_camera_sub = self.create_subscription(
            Image,
            'camera_field',
            self.field_camera_callback,
            10)
        

        self.general_camera_sub
        self.field_camera_sub
        self.state = state
        

    def general_camera_callback(self, image):
        self.state.general_camera_image = self.br.imgmsg_to_cv2(image)

    def field_camera_callback(self, image):
        self.state.field_camera_image = self.br.imgmsg_to_cv2(image)
        cv2.imwrite("recv.png", self.state.field_camera_image)

class StateManager:
    def __init__(self) -> None:
        self.general_camera_image = None
        self.field_camera_image = None

class FlaskApp:
    def __init__(self, state) -> None:
        self.app = Flask(__name__)
        self.state = state

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
            print(content)
            return {"status": True}

        @self.app.route("/api/getl")
        def api_getl():
            return [10, 10, 10, 20, 30, 30]
    
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

    camera_sub = CameraSub(state)
    app = FlaskApp(state)
    
    threading.Thread(target=lambda: app.app.run(port=8080, host="0.0.0.0")).start()

    rclpy.spin(camera_sub)
    camera_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
