import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import numpy as np
import datetime


class CameraReader:

    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self.ret = None
        self.lock = threading.Lock()

    def start_reading(self):
        threading.Thread(target=self.read_loop).start()

    def read_loop(self):
        while True:
            ret, frame = self.cap.read()
            with self.lock:
                self.ret = ret
                self.frame = frame
                

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.pub1_ = self.create_publisher(Image, 'camera_general', 1)
        self.pub2_ = self.create_publisher(Image, 'camera_field', 1)
        self.timer = self.create_timer(0.03, self.timer_callback)  # 10 fps
        self.bridge = CvBridge()

        self.cap1 = cv2.VideoCapture('/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0',cv2.CAP_V4L2 )
        self.cap2 = cv2.VideoCapture('/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0', cv2.CAP_V4L2)


        self.set_camera_properties(self.cap1)
        self.set_camera_properties(self.cap2)
        self.reader1 = CameraReader(self.cap1)
        self.reader2 = CameraReader(self.cap2)
        self.reader1.start_reading()
        self.reader2.start_reading()
        
    def set_camera_properties(self, cap):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
    def add_timestamp(self, img):
        now = datetime.datetime.now()
        timestamp = now.strftime("%H:%M:%S.%f")[:-4]  # Format time as HH:MM:SS.s
        cv2.putText(img, timestamp, (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return img
        
    def timer_callback(self):
        with self.reader1.lock:
            ret1 = self.reader1.ret
            img1 = self.reader1.frame
        with self.reader2.lock:
            ret2 = self.reader2.ret
            img2 = self.reader2.frame

        if not ret1:
            img1 = self.create_error_image()
        if not ret2:
            img2 = self.create_error_image()
        img1 = self.add_timestamp(img1)
        img2 = self.add_timestamp(img2)
        threading.Thread(target=self.publish_image, args=(img1, self.pub1_)).start()
        threading.Thread(target=self.publish_image, args=(img2, self.pub2_)).start()
        
    def create_error_image(self):
        img = np.zeros((480, 640, 3), np.uint8)
        cv2.putText(img, 'Camera not available', (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return img

    def publish_image(self, img, pub):
        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        pub.publish(msg)

    def __del__(self):
        self.cap1.release()
        self.cap2.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
