#!/bin/bash

# Ruta al script canSetUp.sh 
#CAN_SETUP_SCRIPT="/home/ubuntu/catkin_ws/AGV_Scripts/canSetUp.sh"

# Ejecutar canSetUp.sh
#if [ -f "$CAN_SETUP_SCRIPT" ]; then
#	echo "Ejecutando canSetUp.sh..."
#	bash "$CAN_SETUP_SCRIPT"
#else 
#	echo "canSetUp.sh no encontrado. Asegurate que la ruta es correcta."
#	exit 1
#fi 

#Ruta al archivo .launch
LAUNCH_FILE="/home/ubuntu/catkin_ws/src/my_launch_files/launch/my_nodes.launch"

#Ejecutar el archivo .launch
if [ -f "$LAUNCH_FILE" ]; then
	echo "Lanzando my_nodes.launch"
	roslaunch "$LAUNCH_FILE"
else
	echo "my_nodes.launch no encontrado. Asegurate que la ruta es correcta."
	exit 1
fi
