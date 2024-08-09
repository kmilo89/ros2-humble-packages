#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class cmdVelPub(Node):

    def __init__(self):
        super().__init__('cmd_vel_pub')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) 
        timer_period = 1  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1
        #msg.angular.z
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_pub = cmdVelPub()
    rclpy.spin(cmd_vel_pub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()