#!/usr/bin/env python3
import rclpy
import sys
import math
import time
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.executors import ExternalShutdownException

from mrover.msg import IK

class TestIK(Node):
    ik_pub: Publisher
    LINK_CD_LEN = 0.5531735368
    JOINT_B_ANGLE = math.pi / 8
    LINK_BC_LEN = 0.5344417294
    EE_LENGTH = 0.13
    BC_POS_X = LINK_BC_LEN * math.cos(JOINT_B_ANGLE)
    BC_POS_Y = LINK_BC_LEN * math.sin(JOINT_B_ANGLE)
    JOINT_C_MIN = -0.959931
    JOINT_C_MAX = 2.87979
    # THETA_MAX = JOINT_B_ANGLE - 1 * JOINT_C_MIN + 0.16084859151
    # THETA_MIN = JOINT_B_ANGLE - 1 * JOINT_C_MAX + 0.16084859151
    THETA_MAX = -1 * math.pi / 6
    THETA_MIN = JOINT_B_ANGLE - 2.85
    A = (THETA_MIN + THETA_MAX) / 2
    B = (THETA_MAX - THETA_MIN) / 2
    Y_MIN = 0.0
    Y_MAX = 0.4

    def __init__(self):
        super().__init__("test_ik")
        
        self.ik_pub = self.create_publisher(IK, "arm_ik", 1)

        t = 0.0

        while True:
            y = (self.Y_MAX + self.Y_MIN) / 2 + (self.Y_MAX - self.Y_MIN) / 2 * math.sin(t)
            theta = self.A + self.B * math.cos(t)

            target = IK()
            target.target.header.stamp = self.get_clock().now().to_msg()
            target.target.header.frame_id = "arm_base_link" # maybe make relative to joint c to make sure b doesn't move??
            target.target.pose.position.x = self.BC_POS_X + self.LINK_CD_LEN * math.cos(theta) + self.EE_LENGTH
            target.target.pose.position.y = y
            target.target.pose.position.z = self.BC_POS_Y + self.LINK_CD_LEN * math.sin(theta)

            self.get_logger().info("Sending IK command...")
            self.ik_pub.publish(target)
            self.get_logger().info(f"{target.target.pose.position.x}, {target.target.pose.position.y}, {target.target.pose.position.z}")
            self.get_logger().info("Sent!")
            # time.sleep(2)
            input("Press enter to continue")
            t += 0.1

def main():
    try:
        rclpy.init()
        rclpy.spin(TestIK())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)

if __name__ == "__main__":
    main()