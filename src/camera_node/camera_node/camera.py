import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_general')
        self.general_camera_publisher = self.create_publisher(Image, 'camera_general', 10)
        self.field_camera_publisher = self.create_publisher(Image, 'camera_field', 10)

        # OpenCV bridge setup 
        self.br = CvBridge()
        self.real_image = False
        GENERAL_CAPTURE_LINK = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
        FIELD_CAPTURE_LINK = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

        self.general_capture = cv2.VideoCapture(GENERAL_CAPTURE_LINK)
        self.field_capture = cv2.VideoCapture(FIELD_CAPTURE_LINK)

        self.field_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
        self.field_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)

        # self.send_image = cv2.imread("send_me.png")
        self.general_image = np.zeros((720, 1280, 3), dtype=np.int8)
        self.general_image_fallback = cv2.putText(self.general_image, 'General Camera', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 255, 255), 2, cv2.LINE_AA)
        
        self.field_image = np.zeros((720, 1280, 3), dtype=np.int8)
        self.field_image_fallback = cv2.putText(self.field_image, 'Field Camera', (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 
                   5, (255, 255, 255), 2, cv2.LINE_AA)
        while True:
            # We can read images here
            general_camera_status, general_image = self.general_capture.read()
            field_camera_status, field_image = self.field_capture.read()
            if general_camera_status:
                self.general_camera_publisher.publish(self.br.cv2_to_imgmsg(general_image))
            else:
                self.general_camera_publisher.publish(self.br.cv2_to_imgmsg(self.general_image_fallback))

            if field_camera_status:
                self.field_camera_publisher.publish(self.br.cv2_to_imgmsg(field_image))
            else:
                self.field_camera_publisher.publish(self.br.cv2_to_imgmsg(self.field_image_fallback))
                   

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
