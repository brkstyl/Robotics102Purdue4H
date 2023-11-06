#-----------
# Title: Robot102.py
# Author: Gavan Burke
# Date: 11/05/2023
# Description:  This file is the one embedded system file for running
#               the Purdue 4H Robot 102 Mecanum 4 wheeled robot.
#-----------

#-----
# IMPORT SECTION


# machine is used for motor control
import machine
import time
from mpu6050 import MPU6050

#-----

#-----
# VARIABLE DEFINITION SECTION

# Configure the motor control pins for your specific setup
# There are 4 motors
# You'll need to replace these pin numbers with the actual GPIO pins you use.
motor_fl = machine.PWM(machine.Pin(12), freq=1000, duty=0)
motor_fr = machine.PWM(machine.Pin(13), freq=1000, duty=0)
motor_bl = machine.PWM(machine.Pin(14), freq=1000, duty=0)
motor_br = machine.PWM(machine.Pin(15), freq=1000, duty=0)

# Configure the ultrasonic sensor pins for your specific setup.
# Define the GPIO pins for the ultrasonic sensor.  There are 2 sensors.
# You will need 6 variables, replace the pin numbers with the actial GPIO pins you use.
trigger_pin_1 = machine.Pin(16, machine.Pin.OUT)
echo_pin_1 = machine.Pin(17, machine.Pin.IN)
trigger_pin_2 = machine.Pin(18, machine.Pin.OUT)
echo_pin_2 = machine.Pin(19, machine.Pin.IN)

#-----

#-----
# FUNCTION SECTION

# MOTOR Example Usage:
# move_forward(512) -> Moves robot forward at 50% speed
# slide_left(768) -> Slides robot to the left at 75% speed
# turn_right(1023) -> Turns robot right at full speed
# stop() -> Stops the robot

def move_forward(speed):
    motor_fl.duty(speed)
    motor_fr.duty(speed)
    motor_bl.duty(speed)
    motor_br.duty(speed)

def move_backward(speed):
    motor_fl.duty(-speed)
    motor_fr.duty(-speed)
    motor_bl.duty(-speed)
    motor_br.duty(-speed)

def slide_left(speed):
    motor_fl.duty(-speed)
    motor_fr.duty(speed)
    motor_bl.duty(speed)
    motor_br.duty(-speed)

def slide_right(speed):
    motor_fl.duty(speed)
    motor_fr.duty(-speed)
    motor_bl.duty(-speed)
    motor_br.duty(speed)

def turn_left(speed):
    motor_fl.duty(-speed)
    motor_fr.duty(speed)
    motor_bl.duty(-speed)
    motor_br.duty(speed)

def turn_right(speed):
    motor_fl.duty(speed)
    motor_fr.duty(-speed)
    motor_bl.duty(speed)
    motor_br.duty(-speed)

def stop():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_bl.duty(0)
    motor_br.duty(0)


# Ultrasonic Sensor Example Usage:
# distance1 = get_distance(trigger_pin1, echo_pin1)
# distance2 = get_distance(trigger_pin2, echo_pin2)
# if distance1 is not None:
#   print("Sensor 1 - Distance: {:.2f} cm".format(distance1))
# else:
#   print("Sensor 1 - Measurement timed out")
# if distance2 is not None:
#   print("Sensor 2 - Distance: {:.2f} cm".format(distance2))
# else:
#   print("Sensor 2 - Measurement timed out")
# time.sleep(1)  # Delay between measurements

def get_distance():
    # Trigger a pulse to initiate the measurement
    trigger_pin.on()
    time.sleep_us(10)
    trigger_pin.off()

    # Measure the duration of the pulse from the echo pin
    pulse_duration = machine.time_pulse_us(echo_pin, 1, 30000)  # Timeout set to 30,000 microseconds

    # Calculate the distance in centimeters
    if pulse_duration > 0:
        distance_cm = (pulse_duration / 2) / 29.1  # Speed of sound is approximately 343 m/s
        return distance_cm
    else:
        return None  # If no echo is received or measurement times out

#MPU6050 Sensor Example Usage:


# Read accelerometer and gyroscope data
# accel_data = sensor.acceleration
# gyro_data = sensor.gyroscope
# print("Accelerometer (m/s^2):", accel_data)
# print("Gyroscope (rad/s):", gyro_data)
# time.sleep(1)  # Delay between sensor readings


# Initialize the I2C interface for the MPU6050 sensor
def get_MPU():
    i2c = machine.I2C(sda=machine.Pin(21), scl=machine.Pin(22))  # Replace with your actual GPIO pins
    MPUSensor = MPU6050(i2c)

    return MPUSensor
#-----

#-----
# MAIN FUNCTION SECTION

# Main loop
while True:
    try:
        # Move forward at 50% speed (adjust the speed as needed)
        move_forward(512)
        
        # You can add other behaviors or conditions here
        
        # Stop the robot after moving forward
        stop()
        
        # You can add more behaviors or conditions as needed

    except KeyboardInterrupt:
        print("Exiting the main loop.")
        break

#-----



 
