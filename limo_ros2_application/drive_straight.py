import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive


class DriveStraight(Node):
    def __init__(self):
        super().__init__('drive_straight')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('publish_ackermann', False)
        self.declare_parameter('ack_cmd_topic', '/limo/ack_cmd')
        self.declare_parameter('linear_x', 0.2)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('publish_hz', 10.0)

        # Optional e-stop gating (default off for quick tests)
        self.declare_parameter('enable_estop', False)
        self.declare_parameter('estop_topic', 'e_stop')
        self.declare_parameter('stop_on_estop', True)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.publish_ackermann = bool(self.get_parameter('publish_ackermann').value)
        self.ack_cmd_topic = str(self.get_parameter('ack_cmd_topic').value)
        self.linear_x = float(self.get_parameter('linear_x').value)
        self.angular_z = float(self.get_parameter('angular_z').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.enable_estop = bool(self.get_parameter('enable_estop').value)
        self.estop_topic = str(self.get_parameter('estop_topic').value)
        self.stop_on_estop = bool(self.get_parameter('stop_on_estop').value)

        self.estop_active = False

        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.ack_publisher_ = None
        if self.publish_ackermann:
            self.ack_publisher_ = self.create_publisher(AckermannDrive, self.ack_cmd_topic, 10)

        if self.enable_estop:
            self.estop_sub_ = self.create_subscription(Bool, self.estop_topic, self.estop_cb, 10)
            self.estop_sub_  # prevent unused warning

        period = 1.0 / max(1e-3, self.publish_hz)
        self.timer_ = self.create_timer(period, self.timer_cb)

    def estop_cb(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def timer_cb(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z

        if self.enable_estop and self.stop_on_estop and self.estop_active:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        if self.ack_publisher_ is not None:
            ack = AckermannDrive()
            ack.speed = float(msg.linear.x)
            # For straight test we keep steering at 0
            ack.steering_angle = 0.0
            self.ack_publisher_.publish(ack)


def main(args=None):
    rclpy.init(args=args)
    node = DriveStraight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

