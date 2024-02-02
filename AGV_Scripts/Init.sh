#!/bin/bash

# Function to start initialization scripts
start_init_scripts() {
    echo "Starting canSetUp.sh..."
    /home/ubuntu/catkin_ws/AGV_Scripts/canSetUp.sh
    sleep 5
    echo "canSetUp.sh completed."

    echo "Starting motor_init.sh..."
    /home/ubuntu/catkin_ws/AGV_Scripts/motor_init.sh
    sleep 2
    echo "motor_init.sh completed."
}

# Function to start the ROS network in the background
start_ros_network() {
    echo "Starting ROS network..."
    /home/ubuntu/catkin_ws/AGV_Scripts/ros-net-init.sh &
    ROS_PID=$!
}

# Function to execute GPIO script
start_gpio_script() {
    echo "Running GPIO.sh for system indication..."
    /home/ubuntu/catkin_ws/AGV_Scripts/GPIO.sh
    echo "GPIO.sh executed."
}

# Function to handle cleanup and exit gracefully
cleanup_and_exit() {
    echo "CTRL+C received. Stopping ROS network..."
    if [[ -n $ROS_PID ]]; then
        kill -SIGINT $ROS_PID
        wait $ROS_PID 2>/dev/null
    fi
    echo "Exiting script."
    exit
}

# Set trap to catch CTRL+C and call cleanup_and_exit function
trap cleanup_and_exit SIGINT

# Main script execution starts here
echo "Starting initialization scripts..."
start_init_scripts

echo "Starting ROS network..."
start_ros_network

# Add a slight delay to ensure ROS network has started
sleep 5

echo "Executing GPIO.sh..."
start_gpio_script

# After starting the ROS network and executing GPIO.sh, we wait indefinitely
# This loop keeps the script alive until CTRL+C is pressed
while true; do
    sleep 1
done

