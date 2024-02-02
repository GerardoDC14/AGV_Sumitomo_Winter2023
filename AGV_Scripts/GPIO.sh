#!/bin/bash

# Define el número de pin GPIO en la Raspberry Pi para el buzzer
PIN_BUZZER=26  # Pin 26 para el buzzer

# Función para inicializar el pin del buzzer
init_buzzer_pin() {
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

# Función para enviar un pulso al buzzer
pulse_buzzer() {
    echo "Enviando pulso alto a GPIO $1..."
    echo 1 > /sys/class/gpio/gpio$1/value
    sleep 2  # Mantiene el buzzer activo por 2 segundos
    echo "Poniendo GPIO $1 a 0 (bajo)..."
    echo 0 > /sys/class/gpio/gpio$1/value
    echo "Pulso enviado a GPIO $1"
}

# Inicializa el pin del buzzer
init_buzzer_pin $PIN_BUZZER

# Envia un pulso al buzzer
pulse_buzzer $PIN_BUZZER
