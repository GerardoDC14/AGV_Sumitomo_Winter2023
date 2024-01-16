#!/bin/bash
# Script para configurar CAN bus

echo "Starting CAN interface configuration."

# Carga los módulos necesarios para CAN
sudo modprobe can
sudo modprobe can_raw # si también necesitas el módulo can_raw
# sudo modprobe vcan # si quieres configurar un dispositivo CAN virtual

# Asegúrate de que el dispositivo can0 existe
if [ -d /sys/class/net/can0 ]; then
    echo "Device can0 found, proceeding with configuration."

    # Detiene la interfaz can0 antes de la configuración
    sudo ip link set can0 down
    
    # Intenta configurar el bus CAN con un bit-rate de 500Kbit/s
    if sudo ip link set can0 type can bitrate 1000000 restart-ms 100; then
        echo "Bitrate set to 1000000 successfully."
        # Si el bitrate se configura correctamente, levanta la interfaz can0
        sudo ip link set can0 up
        echo "Device can0 is up."
    else
        echo "Failed to set bitrate. Please check your CAN device and bitrate."
    fi
    
    # Configura la longitud de la cola de transmisión
    if sudo ifconfig can0 txqueuelen 10000; then
        echo "Transmission queue length set to 10000 successfully."
    else
        echo "Failed to set transmission queue length."
    fi
else
    echo "CAN device can0 does not exist. Please ensure the CAN device is connected and the correct drivers are loaded."
fi

