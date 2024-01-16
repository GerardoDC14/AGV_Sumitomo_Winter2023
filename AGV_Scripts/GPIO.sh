#!/bin/bash

# Define los números de pin GPIO en la Raspberry Pi
PIN1=20  # Equivalente a GPIO 397 en Jetson
PIN2=16  # Equivalente a GPIO 398 en Jetson

# Función para inicializar un pin
init_pin() {
    if [ ! -e /sys/class/gpio/gpio$1 ]; then
        echo $1 > /sys/class/gpio/export
        echo "GPIO $1 exportado"
    else
        echo "GPIO $1 ya exportado"
    fi
    echo out > /sys/class/gpio/gpio$1/direction
    echo "GPIO $1 configurado como salida"
    echo 0 > /sys/class/gpio/gpio$1/value
    echo "GPIO $1 inicializado a 0 (bajo)"
}

# Función para enviar un pulso
pulse_pin() {
    echo "Enviando pulso alto a GPIO $1..."
    echo 1 > /sys/class/gpio/gpio$1/value
    sleep 2
    echo "Poniendo GPIO $1 a 0 (bajo)..."
    echo 0 > /sys/class/gpio/gpio$1/value
    sleep 1
    echo "Pulso enviado a GPIO $1"
}

# Inicializa los pines
init_pin $PIN1
init_pin $PIN2

# Envía un pulso al PIN1 para reiniciar la ESP32(1)
pulse_pin $PIN1
