# robot_module.py

import RPi.GPIO as GPIO
import smbus
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define the GPIO pins
LED_PIN = 16

# Motor 1 (Left)
IN3 = 19
IN4 = 26
ENA = 13

# Motor 2 (Right)
IN1 = 5
IN2 = 6
ENB = 12

# Ultrasonic Sensor Pins
TRIG = 23
ECHO = 24

# MPU Sensor Pins (not explicitly used here but reserved)
SDA = 2
SCL = 3

# Setup GPIO pins
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Initialize PWM
pwm_a = GPIO.PWM(ENA, 1000)  # 1000Hz
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(50)
pwm_b.start(50)

# I2C bus
bus = smbus.SMBus(1)

# MPU6050 address and registers
MPU6050_Address = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# ADS7830 Command Set
ADS7830_CMD_SD_DIFF = 0x00
ADS7830_CMD_SD_SINGLE = 0x80
ADS7830_CMD_DIFF_CHANNEL_0_1 = 0x00
ADS7830_CMD_DIFF_CHANNEL_2_3 = 0x10
ADS7830_CMD_DIFF_CHANNEL_4_5 = 0x20
ADS7830_CMD_DIFF_CHANNEL_6_7 = 0x30

class ADS7830:
    def __init__(self, address=0x4b):
        self.address = address

    def read_adc(self, channel):
        if channel == 0:
            cmd = ADS7830_CMD_SD_DIFF | ADS7830_CMD_DIFF_CHANNEL_0_1
        elif channel == 1:
            cmd = ADS7830_CMD_SD_DIFF | ADS7830_CMD_DIFF_CHANNEL_2_3
        elif channel == 2:
            cmd = ADS7830_CMD_SD_DIFF | ADS7830_CMD_DIFF_CHANNEL_4_5
        elif channel == 3:
            cmd = ADS7830_CMD_SD_DIFF | ADS7830_CMD_DIFF_CHANNEL_6_7
        else:
            raise ValueError("Channel must be 0–3 for differential reading")

        bus.write_byte(self.address, cmd)
        time.sleep(0.01)
        result = bus.read_byte(self.address)
        return result

def MPU_Init():
    bus.write_byte_data(MPU6050_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU6050_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(MPU6050_Address, CONFIG, 0)
    bus.write_byte_data(MPU6050_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(MPU6050_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_Address, addr)
    low = bus.read_byte_data(MPU6050_Address, addr+1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value

def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.HIGH)

def backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(LED_PIN, GPIO.LOW)

def left():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(LED_PIN, GPIO.HIGH)

def right():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.HIGH)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.LOW)

def set_speed(speed):
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def cleanup():
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
