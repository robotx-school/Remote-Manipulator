import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.command_subscriber = self.create_subscription(
            String,
            'robot_control',
            self.safe_control,
            1)
        self.urx_command_publisher = self.create_publisher(String, 'urx_command', 1)

    def send_command(self, data: dict):
        send_data = String()
        send_data.data = json.dumps(data)
        self.urx_command_publisher.publish(send_data)

        
    def safe_control(self, msg):
        try:
            data = json.loads(msg.data)
            if data and "type" in data and "data" in data: 
                if data["type"] == "movel": # movel
                    # Check limits here
                    self.send_command({"type": "model", "data": data["data"]})
                elif data["type"] == "dashboard": # dashboard command
                    self.send_command({"type": "dashboard", "data": data["data"]})
        except json.decoder.JSONDecodeError:
            pass


def main(args=None):
    rclpy.init(args=args)

    robot_control = RobotControlNode()

    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()