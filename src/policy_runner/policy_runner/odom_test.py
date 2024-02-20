import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class Odom(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        self.publisher = self.create_publisher(Odometry, '/real/odom', 10)
        self.obj_publisher = self.create_publisher(String, '/real/obj_det_pose', 10)
        self.declare_parameter('obj_string',"0.0|0.0|0.0")
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_zed", True)
        self.odom_msg = Odometry()
        self.get_logger().info("\033[92m" + "Odom Publisher Started" + "\033[0m")
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        odom = self.get_parameter("publish_odom").get_parameter_value().bool_value
        zed = self.get_parameter("publish_zed").get_parameter_value().bool_value
        if odom:
            self.publisher.publish(self.odom_msg)
            
        if zed:
            self.obj_publisher.publish(String(data=self.get_parameter('obj_string').get_parameter_value().string_value))
            
        if odom and not zed:
            self.get_logger().info("Publishing Odom")
        elif zed and not odom:
            self.get_logger().info("Publishing Zed")
        elif odom and zed:
            self.get_logger().info("Publishing Odom and Zed")
        else:
            self.get_logger().info("Not Publishing")
        
def main(args=None):
    rclpy.init(args=args)
    odom = Odom()
    rclpy.spin(odom)
    odom.destroy_node()
    rclpy.shutdown()