#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import String, Float32

class ControlNode:
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)

        self.direction_subscriber = rospy.Subscriber('/direction/output', String, self.direction_callback)
        self.velocity_subscriber = rospy.Subscriber('/velocity/output', Float32, self.velocity_callback)

        self.current_direction = "stop"
        self.current_velocity = 0.0
        self.last_direction = None  # Track the last sent direction

    def direction_callback(self, msg):
        if msg.data != self.last_direction:
            self.current_direction = msg.data
            self.last_direction = msg.data
            self.update_direction()

    def velocity_callback(self, msg):
        self.current_velocity = msg.data
        self.update_velocity()

    def update_direction(self):
        direction_command = self.get_direction_command()
        subprocess.run(direction_command, shell=True)
        rospy.loginfo(f"Sending direction command: {direction_command}")

    def update_velocity(self):
        velocity_command = self.get_velocity_command()
        subprocess.run(velocity_command, shell=True)
        rospy.loginfo(f"Sending velocity command: {velocity_command}")

    def get_direction_command(self):
        if self.current_direction == "forward":
            return "cansend can0 602#2F7E600040"
        elif self.current_direction == "backward":
            return "cansend can0 602#2F7E600000"
        else:
            return "cansend can0 602#23FF6000000000"  # Stop command

    def get_velocity_command(self):
        hex_velocity = format(int(self.current_velocity), '04x').upper()
        return f"cansend can0 602#23FF600000{hex_velocity}00"

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    control_node = ControlNode()
    control_node.run()
