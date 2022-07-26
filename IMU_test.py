import RPi.GPIO as GPIO
import smbus
from time import sleep

B1B = 25
A1B = 23
A1A = 12
B1A = 19

GPIO.setmode(GPIO.BCM)

GPIO.setup(A1A, GPIO.OUT)
GPIO.setup(A1B, GPIO.OUT)
GPIO.setup(B1A, GPIO.OUT)
GPIO.setup(B1B, GPIO.OUT)

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

print("1helo :D")
def sensorloop():
    time = 0
    print("2helo :D")
    x = 0
    y = 0
    z = 0
    Dx = 0
    Dy = 0
    Dz = 0
    while 1:
        print("3helo :D")
        acc_x = read_raw_data(ACCEL_XOUT_H)  # Read Accelerometer raw value
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        gyro_x = read_raw_data(GYRO_XOUT_H)  # Read Gyroscope raw value
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        Ax = float(acc_x)/16384.0  # Full scale range +/- 250 degree/C as per sensitivity scale-factor
        Ay = float(acc_y)/16384.0
        Az = float(acc_z)/16384.0

        x += Ax*9.8*0.005
        y += Ay*9.8*0.005
        z += Az*9.8*0.005

        #Gx = float(gyro_x)/131.0
        #Gy = float(gyro_y)/131.0
        #Gz = float(gyro_z)/131.0

        #Dx += Gx*0.005
        #Dy += Gy*0.005
        #Dz += Gz*0.005

#        actuation_power += [dc_cur.power()]
#        computation_power += [pi_cur.power()]
        #timestamps_s += [time]

        sleep(0.25)
        #time += 1

        print("Ax:", str(Ax), " Ay:", str(Ay), " Az:", str(Az))
        #print("Gx:", str(Ax), " Gy:", str(Gy), " Az:", str(Gz))
sensorloop()