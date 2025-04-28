import RPi.GPIO as GPIO
import smbus #import SMBus module of I2C
import time
from time import sleep          #import

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
TRIG = 23 #on zero its 23
ECHO = 24#on zero its 24

#MPU Sensor Pins
SDA = 2
SCL = 3

#Setup GPIO pins
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

#Initialize PWM for motor speed control
pwm_a = GPIO.PWM(ENA, 1000)  # 1000Hz frequency
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(50)  # Start at 50% duty cycle
pwm_b.start(50)

# Variables
safe_distance = 20  # in cm (stop if object is closer than this)
current_distance = 0

def get_distance():
    # Send pulse to trigger
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()

    # Wait for echo to go high
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # Wait for echo to go low
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Calculate distance in cm
    distance = round(distance, 2)

    return distance

bus = smbus.SMBus(1)

# Device addresses
MPU6050_Address = 0x68

# MPU6050 Registers
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
ADS7830_CMD_SD_DIFF = 0x00  # Differential Inputs
ADS7830_CMD_SD_SINGLE = 0x80  # Single-Ended Inputs
ADS7830_CMD_DIFF_CHANNEL_0_1 = 0x00  # +IN = CH0, -IN = CH1
ADS7830_CMD_DIFF_CHANNEL_2_3 = 0x10  # +IN = CH2, -IN = CH3
ADS7830_CMD_DIFF_CHANNEL_4_5 = 0x20  # +IN = CH4, -IN = CH5
ADS7830_CMD_DIFF_CHANNEL_6_7 = 0x30  # +IN = CH6, -IN = CH7
ADS7830_CMD_SNGL_CHANNEL_0 = 0x00  # +IN = CH0, -IN = GND
ADS7830_CMD_SNGL_CHANNEL_1 = 0x10  # +IN = CH1, -IN = GND
ADS7830_CMD_SNGL_CHANNEL_2 = 0x20  # +IN = CH2, -IN = GND
ADS7830_CMD_SNGL_CHANNEL_3 = 0x30  # +IN = CH3, -IN = GND
ADS7830_CMD_SNGL_CHANNEL_4 = 0x40  # +IN = CH4, -IN = GND
ADS7830_CMD_SNGL_CHANNEL_5 = 0x50  # +IN = CH5, -IN = GND
ADS7830_CMD_SNGL_CHANNEL_6 = 0x60  # +IN = CH6, -IN = GND
ADS7830_CMD_SNGL_CHANNEL_7 = 0x70  # +IN = CH7, -IN = GND

# Initialize the MPU6050 sensor
def MPU_Init():
    bus.write_byte_data(MPU6050_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU6050_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(MPU6050_Address, CONFIG, 0)
    bus.write_byte_data(MPU6050_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(MPU6050_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_Address, addr)
    low = bus.read_byte_data(MPU6050_Address, addr+1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

# ADS7830 class to handle analog input readings
class ADS7830:
    def __init__(self, address=0x4b):
        self.address = address

    def read_adc(self, channel):
        """
        Read ADC value from specified differential channel (0–7).
        Differential pairs:
            0: CH0 - CH1
            1: CH2 - CH3
            2: CH4 - CH5
            3: CH6 - CH7
            4: CH1 - CH0
            5: CH3 - CH2
            6: CH5 - CH4
            7: CH7 - CH6
        """
        # Build command based on channel
        if channel == 0:
            cmd = ADS7830_CMD_SD_DIFF | ADS7830_CMD_DIFF_CHANNEL_0_1
        elif channel == 1:
            cmd = ADS7830_CMD_SD_DIFF | ADS7830_CMD_DIFF_CHANNEL_2_3
        elif channel == 2:
            cmd = ADS7830_CMD_SD_DIFF | ADS7830_CMD_DIFF_CHANNEL_4_5
        elif channel == 3:
            cmd = ADS7830_CMD_SD_DIFF | ADS7830_CMD_DIFF_CHANNEL_6_7
        elif channel == 4:
            cmd = ADS7830_CMD_SD_DIFF | 0x40  # CH1 - CH0
        elif channel == 5:
            cmd = ADS7830_CMD_SD_DIFF | 0x50  # CH3 - CH2
        elif channel == 6:
            cmd = ADS7830_CMD_SD_DIFF | 0x60  # CH5 - CH4
        elif channel == 7:
            cmd = ADS7830_CMD_SD_DIFF | 0x70  # CH7 - CH6
        else:
            raise ValueError("Channel must be 0–7")

        # Write command and read ADC value
        bus.write_byte(self.address, cmd)
        time.sleep(0.01)  # Short delay to allow ADC conversion
        result = bus.read_byte(self.address)

        return result

# Initialize both sensors
MPU_Init()
ads = ADS7830()

def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.HIGH)
    print("Moving Forward")

def backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(LED_PIN, GPIO.LOW)
    print("Moving Backward")

def left():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(LED_PIN, GPIO.HIGH)
    print("Turning Left")

def right():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.HIGH)
    print("Turning Right")

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.LOW)
    print("Stopping")

def set_speed(speed):
    # Speed should be between 0-100
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)
    
#Main COde
try:
    while True:
        # Get current distance
        current_distance = get_distance()
        print(f"Distance: {current_distance} cm")

        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        #Full scale range +/- 250 degree/C as per sensitivity scale factor In g units
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0



        print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
        sleep(1)
        
        #Reads position of joystick
        
 
        adc0 = ads.read_adc(0)
        adc1 = ads.read_adc(1)
        print(f"ADC0: {adc0}, ADC1: {adc1}")
        sleep(1)
    '''    if current_distance > safe_distance:
            forward()
        elif current_distance < safe_distance and current_distance > safe_distance/2:
            # Object getting close, slow down
            set_speed(30)
            backward()
            time.sleep(0.5)
            stop()
            time.sleep(0.1)

            # Decide which way to turn
            # First check right
            right()
            time.sleep(0.3)
            stop()
            right_dist = get_distance()

            # Then check left
            left()
            time.sleep(0.6)  # Turn a bit more to the left
            stop()
            left_dist = get_distance()

            # Choose direction with more space
            if right_dist > left_dist and right_dist > safe_distance:
                right()
                time.sleep(0.5)
            elif left_dist > safe_distance:
                left()
                time.sleep(0.5)
            else:
                # No good options, go backward
                backward()
                time.sleep(1)
        else:
            # Too close, emergency stop and reverse
            stop()
            time.sleep(0.1)
            backward()
            time.sleep(1)
            stop()
            '''
    time.sleep(2)  # Short delay between measurements

except KeyboardInterrupt:
    print("Program stopped by user")
    stop()
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()

split this into the main and another file to include it
