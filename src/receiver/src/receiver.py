#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import subprocess

def twist_callback(msg):
    try:
        # Use the linear x-component for motor speed control
        motor_input = msg.linear.x

        # Check the sign of the value to determine the direction command
        if motor_input >= 0:
            # Send command for positive direction
            direction_command = "cansend can0 602#2F7E600040"
        else:
            # Send command for negative direction and use absolute value
            direction_command = "cansend can0 602#2F7E600000"
            motor_input = abs(motor_input)

        # Send the direction command
        subprocess.run(direction_command, shell=True, capture_output=True, text=True)
        rospy.loginfo(f"Sending direction command: {direction_command}")

        # Convert the motor input to an appropriate integer value
        motor_speed = int(motor_input * 600)  # Example scaling

        # Clamp the motor speed to a safe range
        clamped_motor_speed = max(0, min(3000, motor_speed))

        # Convert the motor speed to a hexadecimal value with a fixed length of 4 characters
        hex_motor_speed = format(clamped_motor_speed, '04x').upper()

        # Construct the CAN message for speed control
        speed_command = f"cansend can0 602#23FF600000{hex_motor_speed}00"

        # Log the CAN messages for debugging
        rospy.loginfo(f"Sending speed control command: {speed_command}")

        # Send the CAN message for speed control
        subprocess.run(speed_command, shell=True, capture_output=True, text=True)
    except Exception as e:
        rospy.logerr(f"Error in twist_callback: {e}")

if __name__ == '__main__':
    try:
        rospy.init_node('receiver', anonymous=True)

        # Subscribe to the twist topic
        rospy.Subscriber('/cmd_velA', Twist, twist_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
