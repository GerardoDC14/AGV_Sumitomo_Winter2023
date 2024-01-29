#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import subprocess

def motor_cmd_callback(msg):
    # Scale linear x (0 to 5) to RPM (0 to 3000)
    rpm = msg.linear.x * 600  # 5 maps to 3000, so the scaling factor is 600

    # Construct the CAN message
    # Convert RPM to a hexadecimal value with a fixed length of 4 characters
    hex_rpm = format(int(rpm), '04x')
    can_command = f"cansend can0 602#23FF600000{hex_rpm}0000"

    # Send the CAN message
    subprocess.run(can_command, shell=True)

if __name__ == '__main__':
    rospy.init_node('can_motor_controller', anonymous=True)
    
    # Subscribe to the motor command topic
    rospy.Subscriber('/motor_cmd', Twist, motor_cmd_callback)

    rospy.spin()

