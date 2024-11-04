#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # create ROS subscribers and publishers.
        
        self.TTCThreshold = 1.3
        self.declare_parameter('TTCThreshold', rclpy.Parameter.Type.DOUBLE)

        self.publishDrive = self.create_publisher(
            AckermannDriveStamped, "drive", 10
        )

        self.subscribeScan = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.subscribeOdom = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.odom_callback, 10
        )

    def odom_callback(self, odom_msg):
        # update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # calculate TTC
        minAng = scan_msg.angle_min
        maxAng = scan_msg.angle_max
        incAng = scan_msg.angle_increment

        ranges = np.array(scan_msg.ranges)

        TTC = np.array([np.inf if (max(self.speed*np.cos(minAng+incAng*i), 0)==0) else ranges[i]/(self.speed*np.cos(minAng+incAng*i)) for i in range(len(ranges))])
        print(f"Speed : {self.speed}")
        minTTC = np.min(TTC)
        print(f"minTTC: {minTTC}")
        print(f"arg : {np.argmin(TTC)}")

        # publish command to brake

        if (minTTC < float(self.get_parameter('TTCThreshold').value)):
            driveMessage = AckermannDriveStamped()
            driveMessage.header.stamp = self.get_clock().now().to_msg() 
            driveMessage.header.frame_id = 'base_link'

            driveMessage.drive.speed = 0.
            self.publishDrive.publish(driveMessage)

            
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()