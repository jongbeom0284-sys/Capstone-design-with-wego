import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from ackermann_msgs.msg import AckermannDrive

class LimoControl(Node):
    def __init__(self):
        super().__init__('limo_control')
        
        # Setting for publisher of cmd velocity
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ack_publisher_ = None
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Parameters: speed control + e-stop behavior
        self.declare_parameter('default_speed', 0.2)
        self.declare_parameter('p_gain', 0.01)
        self.declare_parameter('enable_estop', True)
        self.declare_parameter('estop_timeout_s', 0.5)
        # Ackermann output (for limo_ackerman_base)
        self.declare_parameter('publish_ackermann', False)
        self.declare_parameter('ack_cmd_topic', '/limo/ack_cmd')
        self.declare_parameter('steering_gain', 1.0)

        self.default_speed = self.get_parameter('default_speed')
        self.p_gain = self.get_parameter('p_gain')
        self.enable_estop = bool(self.get_parameter('enable_estop').value)
        self.estop_timeout_s = float(self.get_parameter('estop_timeout_s').value)
        self.publish_ackermann = bool(self.get_parameter('publish_ackermann').value)
        self.ack_cmd_topic = str(self.get_parameter('ack_cmd_topic').value)
        self.steering_gain = float(self.get_parameter('steering_gain').value)

        if self.publish_ackermann:
            self.ack_publisher_ = self.create_publisher(AckermannDrive, self.ack_cmd_topic, 10)

        # Setting for subscriber of e_stop and lane_detection
        self.e_stop_subscription = self.create_subscription(
            Bool,
            'e_stop',
            self.e_stop_callback,
            10)
        self.distance_subscription = self.create_subscription(
            Int32,
            'distance_y',
            self.distance_callback,
            10)
        
        # prevent from warning
        self.e_stop_subscription
        self.distance_subscription
        
        # flag and input value of twisting
        # Default to STOP when e-stop enabled, otherwise allow motion.
        self.e_stop_flag = True if self.enable_estop else False
        self.last_estop_msg_time = None
        self.gap = 0

    def e_stop_callback(self, msg):
        self.e_stop_flag = bool(msg.data)
        self.last_estop_msg_time = self.get_clock().now()
    
    def distance_callback(self, msg):
        self.gap = msg.data

    def timer_callback(self):
        # set the limo speed
        msg = Twist()
        msg.linear.x = self.default_speed.value
        msg.angular.z = self.gap * self.p_gain.value

        if self.enable_estop:
            # If we haven't received e_stop recently, fail-safe to STOP.
            if self.last_estop_msg_time is None:
                estop_active = True
            else:
                dt = (self.get_clock().now() - self.last_estop_msg_time).nanoseconds * 1e-9
                estop_active = (dt > self.estop_timeout_s) or self.e_stop_flag

            if estop_active:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                
        self.publisher_.publish(msg)
        if self.ack_publisher_ is not None:
            ack = AckermannDrive()
            ack.speed = float(msg.linear.x)
            ack.steering_angle = float(msg.angular.z) * self.steering_gain
            self.ack_publisher_.publish(ack)
        
def main(args=None):
    rclpy.init(args=args)
    limo_control = LimoControl()

    rclpy.spin(limo_control)

    limo_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()