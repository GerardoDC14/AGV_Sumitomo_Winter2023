#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import subprocess

# Global variables
desired_velocity = 0
direction_active = False  # Tracks if the direction joystick is active
current_direction = 0     # Tracks the current direction from joystick #1

def velocity_callback(msg):
    global desired_velocity
    # Update desired_velocity only if direction joystick is active
    if direction_active:
        dead_zone_threshold = 0.1
        if abs(msg.linear.y) > dead_zone_threshold:
            desired_velocity = max(0, min(10, msg.linear.y * 10))
        else:
            desired_velocity = 0
        send_motor_commands()

def direction_callback(msg):
    global direction_active, current_direction
    dead_zone_threshold = 1
    if abs(msg.linear.y) > dead_zone_threshold:
        direction_active = True
        current_direction = msg.linear.y
    else:
        direction_active = False
        stop_motors()  # Stop motors if direction joystick is not active

def send_motor_commands():
    direction_command_motor1, direction_command_motor2 = get_direction_commands(current_direction)
    send_velocity_command("602", direction_command_motor1)
    send_velocity_command("601", direction_command_motor2)

def get_direction_commands(joystick_value):
    if joystick_value > 0:
        return "cansend can0 602#2F7E600040", "cansend can0 601#2F7E600000"
    else:
        return "cansend can0 602#2F7E600000", "cansend can0 601#2F7E600040"

def send_velocity_command(motor_id, direction_command):
    global desired_velocity
    subprocess.run(direction_command, shell=True, capture_output=True, text=True)
    hex_velocity = format(int(desired_velocity), '04x').upper()
    velocity_command = f"cansend can0 {motor_id}#23FF600000{hex_velocity}00"
    rospy.loginfo(f"Sending velocity control command for motor {motor_id}: {velocity_command}")
    subprocess.run(velocity_command, shell=True, capture_output=True, text=True)

def stop_motors():
    stop_command_motor1 = "cansend can0 602#23FF6000000000"
    stop_command_motor2 = "cansend can0 601#23FF6000000000"
    subprocess.run(stop_command_motor1, shell=True, capture_output=True, text=True)
    subprocess.run(stop_command_motor2, shell=True, capture_output=True, text=True)
    rospy.loginfo("Stopping both motors.")

if __name__ == '__main__':
    try:
        rospy.init_node('receiver', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, velocity_callback)
        rospy.Subscriber('/cmd_dir', Twist, direction_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
