import RPi.GPIO as GPIO
import smbus
import board
import busio
from time import sleep
import matplotlib.pyplot as plt
import adafruit_ina219
import numpy as np

B1B = 25
A1B = 23
A1A = 12
B1A = 19

GPIO.setmode(GPIO.BCM)

GPIO.setup(A1A, GPIO.OUT)
GPIO.setup(A1B, GPIO.OUT)
GPIO.setup(B1A, GPIO.OUT)
GPIO.setup(B1B, GPIO.OUT)

GPIO.output(B1B, GPIO.LOW)
#GPIO.output(A1A, GPIO.LOW)
GPIO.output(A1B, GPIO.LOW)

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

i2c = busio.I2C(board.SCL, board.SDA)
print(i2c)
dc_ina219 = adafruit_ina219.INA219(i2c, 0x40)
pi_ina219 = adafruit_ina219.INA219(i2c, 0x44)


def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def MPU_offset():
    a_x = 0
    a_y = 0
    a_z = 0
    g_x = 0
    g_y = 0
    g_z = 0

    for i in range(25):
        raw_data = read_raw_data()
        a_x += raw_data[0]
        a_y += raw_data[1]
        a_z += raw_data[2]

        g_x += raw_data[4]
        g_y += raw_data[5]
        g_z += raw_data[6]
        sleep(0.01)

    return [(a_x/25),(a_y/25),(a_z/25),0,(g_x/25),(g_y/25),(g_z/25),]


def read_raw_data():
    # output list ax, ay, az, temp, gx, gy, gz
    fourteen = bus.read_i2c_block_data(0x68, 0x3b, 14)

    his, los   = fourteen[0::2], fourteen[1::2]
    values = []
    for hi, lo in zip(his, los):
        value = (hi << 8) + lo
        if value > 0x8000:
            value = -((65535-value) +1)
        values.append(value)
    return values

bus = smbus.SMBus(1)
Device_Address = 0x68
MPU_Init()

print("1helo :D")
def sensorloop():

    time = 0

    Ax_list = []
    Ay_list = []
    Az_list = []
    Gx_list = []
    Gy_list = []
    Gz_list = []
    x_list = []
    y_list = []
    z_list = []
    speed_list = []
    pi_power_list = []
    dc_power_list = []


    t=np.arange(0.0, 4.0, 0.05)
    offset = MPU_offset()

    print("2helo :D")
    xyz = [0,0,0]
    xyz_prev = [0,0,0]
    xyz_array = np.array(xyz)
    xyz_prev_array = np.array(xyz_prev)

    dx = 0
    dy = 0
    dz = 0
    speed = 0

    Dx = 0
    Dy = 0
    Dz = 0

    pwm1 = GPIO.PWM(B1A, 1000)
    pwm2 = GPIO.PWM(A1A, 1000)

    pwm1.start(00)
    pwm2.start(00)
    GPIO.output(B1B, GPIO.HIGH)
    GPIO.output(A1B, GPIO.HIGH)

    for i in range(80):
        try:
            print("3helo :D")
            pwm1.ChangeDutyCycle((100*(i/80)))
            pwm2.ChangeDutyCycle((100*(i/80)))

            pi_bus_voltage = pi_ina219.bus_voltage
            dc_bus_voltage = dc_ina219.bus_voltage
            pi_shunt_voltage = pi_ina219.shunt_voltage
            dc_shunt_voltage = dc_ina219.shunt_voltage

            pi_current = pi_ina219.current
            dc_current = dc_ina219.current
            pi_power = pi_ina219.power
            dc_power = dc_ina219.power

            raw_data = read_raw_data()
            acc_x = raw_data[0] - offset[0]
            acc_y = raw_data[1] - offset[1]
            acc_z = raw_data[2] - offset[2]

            gyro_x = raw_data[4] - offset[4]
            gyro_y = raw_data[5] - offset[5]
            gyro_z = raw_data[6] - offset[6]

            Ax = float(acc_x)/16384.0  # Full scale range +/- 250 degree/C as per sensitivity scale-factor
            Ay = float(acc_y)/16384.0
            Az = float(acc_z)/16384.0

            xyz[0] += Ax*9.8*0.005
            xyz[1] += Ay*9.8*0.005
            xyz[2] += Az*9.8*0.005

            speed = (((xyz[0]-xyz_prev[0])**2 + (xyz[1]-xyz_prev[1])**2 + (xyz[2]-xyz_prev[2])**2)**(1/2))/0.005
            speed_list += [speed]

            Gx = float(gyro_x)/131.0
            Gy = float(gyro_y)/131.0
            Gz = float(gyro_z)/131.0

            Dx += Gx*0.005
            Dy += Gy*0.005
            Dz += Gz*0.005

            Ax_list += [Ax]
            Ay_list += [Ay]
            Az_list += [Az]

            Gx_list += [Gx]
            Gy_list += [Gy]
            Gz_list += [Gz]

            xyz_prev = xyz
        except:
            print("BITCH")

        sleep(0.05)
        time += 50

        print(str(acc_x))
        print("Ax:", str(Ax), " Ay:", str(Ay), " Az:", str(Az))
        print("Gx:", str(Ax), " Gy:", str(Gy), " Az:", str(Gz))
        pi_power_list += [(pi_bus_voltage) * (pi_current/1000)]
        dc_power_list += [(dc_bus_voltage) * (dc_current/1000)]
        print("Pi power calc. : {:8.5f} W".format(pi_bus_voltage * (pi_current/1000) ))
        print("Motor power calc. : {:8.5f} W".format(dc_bus_voltage * (dc_current/1000) ))
    figure, data = plt.subplots(3,3)

    data[0,0].plot(t, Ax_list)
    data[0,0].set_title("Ax vs t(ms)")
    data[0,1].plot(t, Ay_list)
    data[0,1].set_title("Ay vs t(ms)")
    data[0,2].plot(t, Az_list)
    data[0,2].set_title("Az vs t(ms)")
    data[1,0].plot(t, Gx_list)
    data[1,0].set_title("Gx vs t(ms)")
    data[1,1].plot(t, Gy_list)
    data[1,1].set_title("Gy vs t(ms)")
    data[1,2].plot(t, Gz_list)
    data[1,2].set_title("Gz vs t(ms)")
    data[2,0].plot(t, speed_list)
    data[2,0].set_title("t vs speed")
    data[2,1].plot(speed_list, dc_power_list)
    data[2,1].set_title("v(m/a) vs Pa(W)")
    data[2,2].plot(t, dc_power_list)
    data[2,2].set_title("t vs actuation power")

    plt.show()

sensorloop()

GPIO.cleanup()