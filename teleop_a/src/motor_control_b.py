#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String

import RPi.GPIO as GPIO

#Set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Set variables for GPIO Pin Motors
MOTOR_P=17
MOTOR_N=22
MOTOR_ENABLE=25
MOTOR_RP=23
MOTOR_RN=24
MOTOR_RENABLE=26
FREQUENCY=1000

#Set GPIO pins to be outputs
GPIO.setup(MOTOR_P,GPIO.OUT)
GPIO.setup(MOTOR_N,GPIO.OUT)
GPIO.setup(MOTOR_ENABLE,GPIO.OUT)
GPIO.setup(MOTOR_RP,GPIO.OUT)
GPIO.setup(MOTOR_RN,GPIO.OUT)
GPIO.setup(MOTOR_RENABLE,GPIO.OUT)

#Set pwm controller and start it
pwm=GPIO.PWM(MOTOR_ENABLE,FREQUENCY)
pwmR=GPIO.PWM(MOTOR_RENABLE,FREQUENCY)
pwm.start(0)
pwmR.start(0)
GPIO.output(MOTOR_P,GPIO.HIGH)
GPIO.output(MOTOR_N,GPIO.LOW)
GPIO.output(MOTOR_ENABLE,GPIO.HIGH)
GPIO.output(MOTOR_RP,GPIO.HIGH)
GPIO.output(MOTOR_RN,GPIO.LOW)
GPIO.output(MOTOR_RENABLE,GPIO.HIGH)

#Message Handler
def left_callback(data):

    hi=data.data
    if data.data < 0:
        GPIO.output(MOTOR_P,GPIO.LOW)
        GPIO.output(MOTOR_N,GPIO.HIGH)
        hi=-hi
    else:
        GPIO.output(MOTOR_P,GPIO.HIGH)
        GPIO.output(MOTOR_N,GPIO.LOW)
    pwm.ChangeDutyCycle(hi)

#Message Handler
def right_callback(data):

    hello=data.data
    if data.data < 0:
        GPIO.output(MOTOR_RN,GPIO.LOW)
        GPIO.output(MOTOR_RP,GPIO.HIGH)
        hello=-hello
    else:
        GPIO.output(MOTOR_RN,GPIO.HIGH)
        GPIO.output(MOTOR_RP,GPIO.LOW)
    pwmR.ChangeDutyCycle(hello)

#Initialize as ROS node
rospy.init_node('motor_control_b')
rospy.Subscriber("left_control",Float64,left_callback)
rospy.Subscriber("right_control",Float64,right_callback)
# Ready to go
rospy.loginfo("Motor Control B initialized...")

# Loop continuously
rospy.spin()

GPIO.output(MOTOR_ENABLE,GPIO.LOW)
pwm.stop()
GPIO.output(MOTOR_RENABLE,GPIO.LOW)
pwmR.stop()
GPIO.cleanup()

