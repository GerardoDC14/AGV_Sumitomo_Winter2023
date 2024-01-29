#!/bin/bash

#Enviar comandos CANopen para inicializar el motor

cansend can0 602#2F60600003
sleep 1
cansend can0 601#2F60600003
sleep 1

cansend can0 602#2B4060000600
sleep 1
cansend can0 601#2B4060000600
sleep 1

cansend can0 602#2B4060000700
sleep 1
cansend can0 601#2B4060000700
sleep 2

cansend can0 602#2B4060000F00
sleep 1
cansend can0 601#2B4060000F00
sleep 1



