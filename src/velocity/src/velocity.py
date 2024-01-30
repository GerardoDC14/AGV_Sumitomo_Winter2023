#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32  # Using Float32 for velocity

class VelocityNode:
    def __init__(self):
        rospy.init_node('velocity_node', anonymous=True)

        self.joystick_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.joystick_callback)
        self.velocity_publisher = rospy.Publisher('/velocity/output', Float32, queue_size=10)

        self.dead_zone_threshold = 0.1

    def joystick_callback(self, msg):
        if abs(msg.linear.y) > self.dead_zone_threshold:
            # Scale the input from 0 to 1000 RPM
            velocity = max(0.0, min(1000, msg.linear.y * 100))
        else:
            velocity = 0.0

        self.velocity_publisher.publish(Float32(velocity))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    velocity_node = VelocityNode()
    velocity_node.run()
