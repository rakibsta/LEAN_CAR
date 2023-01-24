# Rakibul Chowdhury
# rakibul@mit.edu
# 14 July 2022
# PID Controller using MPU6050 IMU and TB6612FNG h-bridge
# MPU6050 code from electronicwings.com

import RPI.GPIO as GPIO        # GPIO module
import smbus                   # SMBus module I2C
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
import curses
from time import sleep

B1B = 25
A1B = 23
A1A = 12
B1A = 19

GPIO.setmode(GPIO.BCM)

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(STBY, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)

screen = curses.initscr()
curses.noecho() 
curses.cbreak()
screen.keypad(True)

try:
        while True:   
            char = screen.getch()
            if char == ord('q'):
                break
            elif char == curses.KEY_UP:
                GPIO.output(25,False)
                GPIO.output(19,True)
                GPIO.output(23,False)
                GPIO.output(12,True)
            elif char == curses.KEY_DOWN:
                GPIO.output(25,True)
                GPIO.output(19,False)
                GPIO.output(23,True)
                GPIO.output(12,False)
            elif char == curses.KEY_RIGHT:
                GPIO.output(25,True)
                GPIO.output(19,False)
                GPIO.output(23,False)
                GPIO.output(12,True)
            elif char == curses.KEY_LEFT:
                GPIO.output(25,False)
                GPIO.output(19,True)
                GPIO.output(23,True)
                GPIO.output(12,False)
            elif char == 10:
                GPIO.output(25,False)
                GPIO.output(19,False)
                GPIO.output(23,False)
                GPIO.output(12,False)
             
finally:
    #Close down curses properly, inc turn echo back on!
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
    GPIO.cleanup()