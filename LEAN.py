# Rakibul Chowdhury
# rakibul@mit.edu
# 14 July 2022
# PID Controller using MPU6050 IMU and TB6612FNG h-bridge
# MPU6050 code from electronicwings.com

import RPI.GPIO as GPIO        # GPIO module
import smbus                   # SMBus module I2C
from ina219 import INA219      # INA219 module I2C
from time import sleep

GPIO.setmode(GPIO.BOARD)

BIN1 = 23
BIN2 = 24
STBY = 25
AIN1 = 22
AIN2 = 26
PWMB = 1
PWMA = 5

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

pi_cur = INA219(shunt_ohms=0.1,
                max_expected_amps=0.4,
                address=0x40)
pi_cur.configure(voltage_range=ina.RANGE_16V,
                 gain=ina.GAIN_AUTO,
                 bus_adc=ina.ADC_128SAMP
                 shunt_adc=ina.ADC_128SAMP)

dc_cur = INA219(shunt_ohms=0.1, max_expected_amps=0.2, address=0x40)
dc_cur.configure(voltage_range=ina.RANGE_16V,
                 gain=ina.GAIN_AUTO,
                 bus_adc=ina.ADC_128SAMP
                 shunt_adc=ina.ADC_128SAMP)

def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    value = ((high << 8) | low)
    if(value > 32768):
        value = value - 65536
        return value


bus = smbus.SMBus(1)
Device_Address = 0x68
MPU_Init()

def limit(num, minimum, maximum):
    return max(min(num, maximum), minimum)

def sensorloop():
    while true:
        acc_x = read_raw_data(ACCEL_XOUT_H)  # Read Accelerometer raw value
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        gyro_x = read_raw_data(GYRO_XOUT_H)  # Read Gyroscope raw value
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        Ax = acc_x/16384.0  # Full scale range +/- 250 degree/C as per sensitivity scale-factor
        Ay = acc_y/16384.0
        Az = acc_z/16384.0

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0

        actuation_power = dc_cur.power()
        computation_power = pi_cur.power()

        time.sleep(1)

def PIDloop():
    float target = 0.3
    float pos = 0
    float kp = 0.2
    float ki = 0
    float kd = 0
    float error = 0
    float prevError = 0
    float p = 0
    float i = 0
    float d = 0
    float t = 0
    float final = 0

    float target_T = 0
    float theta = 0
    float kp_T = 0.2
    float ki_T = 0
    float kd_T = 0
    float error_T = 0
    float prevError_T = 0
    float p_T = 0
    float i_T = 0
    float d_t = 0
    float final_T = 0

    float final_L = 0
    float final_R = 0

    dc_pwm1 = GPIO.PWM(PWMA, 50)
    dc_pwm2 = GPIO.PWM(PWMB, 50)
    dc_pwm1.start(0)
    dc_pwm2.start(0)
    while true:
        error_T = target_T - theta
        if t = 0:
            prevError_T = errorT
        p_t = kp_T*error_T
        i_T += error_T*t
        d_T = (error_T-prevError_T)/0.005
        final_T = p_T+i_T+d_T

        error = target - pos
        if t = 0:
            prevError = error
        p = kp*error
        i += error*t
        d = (error-prevError)/0.005
        final = kp*p + ki*i + kd*d
        if final > 0:
            GPIO.output(AIN1.HIGH)
            GPIO.output(AIN2.LOW)
            GPIO.output(BIN1.HIGH)
            GPIO.output(BIN2.LOW)
        if final < 0:
            GPIO.output(AIN1.LOW)
            GPIO.output(AIN2.HIGH)
            GPIO.output(BIN1.LOW)
            GPIO.output(BIN2.HIGH)

        duty1 = abs(final) + final_T
        duty2 = abs(final) - final_T

        dc_pwm1.ChangeDutyCycle(limit(duty, 0, 100))
        dc_pwm2.ChangeDutyCycle(limit(duty, 0, 100))

        prevError = error
        t += 0.005
        time.sleep(0.005)

PIDthread = threading.Thread(target=PIDloop)
IMUthread = threading.Thread(target=sensorloop)
