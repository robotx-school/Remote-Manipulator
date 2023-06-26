import numpy as np
import cv2
import socket
import struct
import pickle



class CameraClient:
    def __init__(self, ip: str, port: int, use_fallback: bool = False) -> None:
        self.ip = ip
        self.port = port
        self.image = None  # Latest image stored here
        self.use_fallback = use_fallback
        if self.use_fallback:
            self.fallback_image = cv2.imread("abc.png")
        else:
            self.client_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((ip, port))
            self.payload_size = struct.calcsize("Q")
            self.data = b""

    def get_frame(self) -> np.ndarray:
        if not self.use_fallback:
            while len(self.data) < self.payload_size:
                packet = self.client_socket.recv(4 * 1024)
                if not packet:
                    break
                self.data += packet
            packed_msg_size = self.data[:self.payload_size]
            self.data = self.data[self.payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(self.data) < msg_size:
                self.data += self.client_socket.recv(4 * 1024)
            frame_data = self.data[:msg_size]
            self.data = self.data[msg_size:]
            frame = pickle.loads(frame_data)
            return frame
        else:
            self.image = self.fallback_image
            return self.fallback_image


    def close(self) -> None:
        if not self.use_fallback:
            self.client_socket.close()


if __name__ == "__main__":
    camera = CameraClient("localhost", 9988)
    while True:
        frame = camera.get_frame()
        print(frame.shape)
        #cv2.imwrite("img.png", frame)
        cv2.imshow("img", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    camera.close()
