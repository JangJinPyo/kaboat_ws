from numpy import arccos, arcsin, cos, sin
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import time

LIN_MAX = 0.22
ANG_MAX = 2.84
MAX_SLICE = 8

Target_X = 0.0
Target_Y = 0.0
Threshold = 1.0

class Hbmove(Node):
    def __init__(self):
        super().__init__("move_turtlenot")  # type: ignore
        self.create_timer(0.1, self.turtle_callback)
        self.create_timer(1 / 60, self.update)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile_sensor_data
        )
        self.scan = LaserScan()
        self.scan_avg = [0.0 for _ in range(MAX_SLICE)]
        self.msg = Twist()
        self.clock = self.get_clock()

    def update(self):
        # update variables self.msg, self.scan, self.camera
        if sum([self.scan_avg[0], self.scan_avg[7]]) < 1:
            self.msg.linear.x = 0.0
        else:
            self.msg.linear.x = LIN_MAX
        self.get_logger().info(f"Targeting Location (x,y) = ({Target_X},{Target_Y})")

    def turtle_callback(self):
        self.msg = self.speed_limit(self.msg)
        self.pub.publish(self.msg)

    def speed_limit(self, msg: Twist):
        if msg.linear.x > LIN_MAX:
            msg.linear.x = LIN_MAX
        elif msg.linear.x < -LIN_MAX:
            msg.linear.x = -LIN_MAX
        if msg.angular.z > ANG_MAX:
            msg.angular.z = ANG_MAX
        elif msg.angular.z < -ANG_MAX:
            msg.angular.z = -ANG_MAX
        return msg

    def scan_callback(self, msg: LaserScan):
        self.scan = msg
        for i in range(0,360):
            if msg.ranges[i] > 12.0:
                msg.ranges[i] = 12.0
            if i+1 >= 360:
                msg.ranges[i+1] = msg.ranges[359]
                if abs(msg.ranges[i] - msg.ranges[i+1]) > Threshold:
                    Target_X = cos(i) * msg.ranges[i]
                    Target_Y = sin(i) * msg.ranges[i]

def main():
    rclpy.init()
    node = Hbmove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()