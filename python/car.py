import sys

sys.path.append('/usr/local/lib/python2.7/site-packages')

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import serial
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import spidev
import math
import carFunctions

GPIO.setmode(GPIO.BCM)

pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]
radio = NRF24 (GPIO, spidev.SpiDev())
radio.begin(0,17)
radio.setPayloadSize(1)
radio.setChannel(0x75)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()
radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1, pipes[1])

carFunctions.findCar()