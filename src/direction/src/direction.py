#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # or a more suitable message type

class DirectionNode:
    def __init__(self):
        # Initialize the ROS Node
        rospy.init_node('direction_node', anonymous=True)

        # Subscriber to the joystick topic
        self.joystick_subscriber = rospy.Subscriber('/cmd_dir', Twist, self.joystick_callback)

        # Publisher for the processed direction data
        self.direction_publisher = rospy.Publisher('/direction/output', String, queue_size=10)

        # Dead zone threshold
        self.dead_zone_threshold = 0.2

    def joystick_callback(self, msg):
        # Process joystick data
        direction_data = "stop"  # Default state
        if abs(msg.linear.y) > self.dead_zone_threshold:
            if msg.linear.y > 0:
                direction_data = "forward"
            else:
                direction_data = "backward"
        
        # Publish the direction data
        self.direction_publisher.publish(direction_data)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    direction_node = DirectionNode()
    direction_node.run()
