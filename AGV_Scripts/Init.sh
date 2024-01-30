#!/bin/bash

# Pin de monitoreo (PNL en Jetson)
MONITOR_PIN=21 

# Exporta y configura el GPIO como entrada
if [ ! -e /sys/class/gpio/gpio$MONITOR_PIN ]; then
	echo $MONITOR_PIN > /sys/class/gpio/export
	echo "GPIO $MONITOR_PIN exportado"
fi
echo in > /sys/class/gpio/gpio$MONITOR_PIN/direction
echo "GPIO $MONITOR_PIN configurado como entrada"

# Configuracion de pin para pull-down (0)
#echo "$MONITOR_PIN" > /sys/class/gpio/export
#echo "in" >/sys/class/gpio/gpio"$MONITOR_PIN"/direction
#echo "both" > /sys/class/gpio/gpio"$MONITOR_PIN"/edge

# Inicializacion de shutdown.sh
#/home/ubuntu/catkin_ws/AGV_Scripts/shutdown.sh

# Inicializacion de canSetUp.sh y GPIO.sh
start_init_scripts() {
	/home/ubuntu/catkin_ws/AGV_Scripts/canSetUp.sh &
	/home/ubuntu/catkin_ws/AGV_Scripts/GPIO.sh &
	
	sleep 5

	/home/ubuntu/catkin_ws/AGV_Scripts/motor_init.sh &
	wait
}

# Inicializacion de red ros
start_ros_network() {
	/home/ubuntu/catkin_ws/AGV_Scripts/ros-net-init.sh
}


# Bucle infinito para monitorear el pin 

while true
do
	#Lee el estado del pin GPIO
	read -r PIN_VALUE < /sys/class/gpio/gpio"$MONITOR_PIN"/value
	if [ "$PIN_VALUE" -eq 1 ]; then
		# Pin alto, ejecuta los scripts de inicializacion y luego inicia ROS
		start_init_scripts
		start_ros_network
		break
	fi
	sleep 1
done
