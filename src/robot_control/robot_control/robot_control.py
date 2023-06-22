import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json



class Limits:
    def __init__(self, gripper_max: int, gripper_min: int, x_min: int, x_max: int, y_min: int, y_max: int, z_min: int, z_max: int, vel: int, acc: int):
        self.gripper_max = gripper_max
        self.gripper_min = gripper_min

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max

        self.vel = vel
        self.acc = acc

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

        self.limits = Limits(self.get_parameter("gripper.max").value, self.get_parameter("gripper.min").value, 
                            self.get_parameter("movel.x_min").value, self.get_parameter("movel.x_max").value, 
                            self.get_parameter("movel.y_min").value, self.get_parameter("movel.y_max").value,
                            self.get_parameter("movel.z_min").value, self.get_parameter("movel.z_max").value,
                            self.get_parameter("velocity").value, self.get_parameter("acceleration").value)

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
            self.get_logger().info(f"{data}")
            if data and "type" in data: 
                if data["type"] == "movel": # movel
                    # Check limits here
                    coords = data["data"]
                    can_move = False
                    if (self.limits.z_min <= coords[2] <= self.limits.z_max) and (self.limits.x_min <= coords[0] <= self.limits.x_max) and (self.limits.y_min <= coords[1] <= self.limits.y_max): 
                        can_move = True

                    if can_move:
                        self.send_command({"type": "movel", "data": data["data"], "velocity": self.limits.vel, "acceleration": self.limits.acc})
                elif data["type"] == "dashboard": # dashboard command
                    self.send_command({"type": "dashboard", "data": data["data"], "extra": data["extra"] if "extra" in data else ""})
                elif data["type"] == "gripper":
                    self.send_command({"type": "gripper", "data": data["data"]})
                else:
                    self.send_command(data)

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
