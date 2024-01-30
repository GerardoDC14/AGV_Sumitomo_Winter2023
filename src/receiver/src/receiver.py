#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import subprocess

# Global variable to store the desired velocity
desired_velocity = 0

def velocity_callback(msg):
    global desired_velocity
    # Implementing a dead zone for Joystick #2 (Slider-like behavior)
    dead_zone_threshold = 0.1
    if abs(msg.linear.y) > dead_zone_threshold:
        # Scale the input from 0 to 10
        desired_velocity = max(0, min(10, msg.linear.y * 10))
    else:
        # Reset desired velocity to 0 when in dead zone
        desired_velocity = 0

def direction_callback(msg):
    # Implementing a dead zone for Joystick #1 (Switch-like behavior)
    dead_zone_threshold = 0.3
    if abs(msg.linear.y) > dead_zone_threshold:
        # Determine direction and send velocity command only when outside dead zone
        direction_command_motor1, direction_command_motor2 = get_direction_commands(msg.linear.y)
        send_velocity_command("602", direction_command_motor1)
        send_velocity_command("601", direction_command_motor2)
    else:
        # Stop both motors when Joystick #1 is in the dead zone
        stop_motors()

def get_direction_commands(joystick_value):
    # Determine the direction commands for both motors based on Joystick #1 position
    if joystick_value > 0:
        return "cansend can0 602#2F7E600040", "cansend can0 601#2F7E600000"
    else:
        return "cansend can0 602#2F7E600000", "cansend can0 601#2F7E600040"

def send_velocity_command(motor_id, direction_command):
    global desired_velocity
    # Send the direction command
    subprocess.run(direction_command, shell=True, capture_output=True, text=True)

    # Convert the desired velocity to a hexadecimal value with a fixed length of 4 characters
    hex_velocity = format(int(desired_velocity), '04x').upper()

    # Construct the CAN message for velocity control
    velocity_command = f"cansend can0 {motor_id}#23FF600000{hex_velocity}00"
    rospy.loginfo(f"Sending velocity control command for motor {motor_id}: {velocity_command}")

    # Send the CAN message for velocity control
    subprocess.run(velocity_command, shell=True, capture_output=True, text=True)

def stop_motors():
    # Stop commands for both motors
    stop_command_motor1 = "cansend can0 602#23FF6000000000"
    stop_command_motor2 = "cansend can0 601#23FF6000000000"
    subprocess.run(stop_command_motor1, shell=True, capture_output=True, text=True)
    subprocess.run(stop_command_motor2, shell=True, capture_output=True, text=True)
    rospy.loginfo("Stopping both motors.")

if __name__ == '__main__':
    try:
        rospy.init_node('receiver', anonymous=True)

        # Subscribe to the velocity and direction topics
        rospy.Subscriber('/cmd_vel', Twist, velocity_callback)
        rospy.Subscriber('/cmd_dir', Twist, direction_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
