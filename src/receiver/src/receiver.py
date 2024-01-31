#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import subprocess

# Global variables
desired_velocity = 0  # This will now represent velocity in RPM (0 to 1000)
direction_active = False  # Tracks if the direction joystick is active
current_direction = 0     # Tracks the current direction from joystick #1

def velocity_callback(msg):
    global desired_velocity
    # Scale joystick input (0 to 10) to RPM range (0 to 100)
    dead_zone_threshold = 0.1
    if abs(msg.linear.y) > dead_zone_threshold:
        desired_velocity = int(msg.linear.y * 50)  # Scale up to 100 RPM
    else:
        desired_velocity = 0
        stop_motors()  # Stop motors if velocity joystick is in the dead zone

    send_motor_commands()

def direction_callback(msg):
    global direction_active, current_direction
    dead_zone_threshold = 0.3
    if abs(msg.linear.y) > dead_zone_threshold:
        direction_active = True
        current_direction = msg.linear.y
    else:
        direction_active = False
        # Do not stop motors here; let the velocity joystick handle stopping

def send_motor_commands():
    if direction_active:
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
    
    # Convert the desired velocity to a hexadecimal value in little-endian format
    hex_velocity = format(desired_velocity, '04x')
    little_endian_hex_velocity = hex_velocity[2:] + hex_velocity[:2]
    
    # Construct the CAN message for velocity control
    velocity_command = f"cansend can0 {motor_id}#23FF600000{little_endian_hex_velocity}00"
    rospy.loginfo(f"Sending velocity control command for motor {motor_id}: {velocity_command}")

    # Send the CAN message for velocity control
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
