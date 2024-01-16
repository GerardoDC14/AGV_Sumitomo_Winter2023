#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

def pin_cmd_callback(msg):
    GPIO.output(12, msg.data)

if __name__ == '__main__':
    rospy.init_node('can_sender', anonymous=True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(12, GPIO.OUT)
    rospy.Subscriber('/pin_cmd', Bool, pin_cmd_callback)
    rospy.spin()
    GPIO.cleanup()

