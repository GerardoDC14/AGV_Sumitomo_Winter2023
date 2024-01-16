#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

def btn_press_callback(msg):
    pub.publish(msg.data)

if __name__ == '__main__':
    rospy.init_node('receiver', anonymous=True)
    rospy.Subscriber('/btn_press', Bool, btn_press_callback)
    pub = rospy.Publisher('/pin_cmd', Bool, queue_size=10)
    rospy.spin()
