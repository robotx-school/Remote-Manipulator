import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control')


        # Declare node params with defaults
        self.declare_parameter('gripper.max', 100)
        self.declare_parameter('gripper.min', 0)

        self.declare_parameter('movel.x_min', -0.25)
        self.declare_parameter('movel.x_max', 0.25)
        self.declare_parameter('movel.y_min', -0.150)
        self.declare_parameter('movel.y_max', -0.5)
        self.declare_parameter('movel.z_min', 0.01)
        self.declare_parameter('movel.z_max', 0.2)

        self.declare_parameter('velocity', 0.15)
        self.declare_parameter('acceleration', 0.15)

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
                    self.send_command({"type": "model", "data": data["data"], "velocity": self.get_parameter("velocity").value, "acceleration": self.get_parameter("acceleration").value})
                elif data["type"] == "dashboard": # dashboard command
                    self.send_command({"type": "dashboard", "data": data["data"], "extra": data["extra"] if "extra" in data else ""})
                elif data["type"] == "gripper":
                    self.send_command({"type": "gripper", "data": data["data"]})
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
