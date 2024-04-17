#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
from datetime import datetime
import math
# create a default object, no changes to I2C address or frequency
mh = Raspi_MotorHAT(addr=0x60)

Motor_left=mh.getMotor(1)
Motor_right=mh.getMotor(2)


def callback(data):
    if (data.data):
        if (data.data[0]==1):
            rospy.loginfo("I detcet  %s is STOP", data.data[0])
            Motor_left.setSpeed(0)
            Motor_right.setSpeed(0)
            Motor_left.run(Raspi_MotorHAT.FORWARD)
            Motor_right.run(Raspi_MotorHAT.FORWARD)

    if (data.data):
        if (data.data[0]==2):
            rospy.loginfo("I detcet  %s is FORWARD", data.data[0])
            Motor_left.setSpeed(80)
            Motor_right.setSpeed(80)
            Motor_left.run(Raspi_MotorHAT.FORWARD)
            Motor_right.run(Raspi_MotorHAT.FORWARD)

    if (data.data):
        if (data.data[0]==3):
            rospy.loginfo("I detcet  %s is TURN RIGHT", data.data[0])
            Motor_left.setSpeed(80)
            Motor_right.setSpeed(80)
            Motor_left.run(Raspi_MotorHAT.FORWARD)
            Motor_right.run(Raspi_MotorHAT.BACKWARD)
            rospy.sleep(2.)
            Motor_left.run(Raspi_MotorHAT.FORWARD)
            Motor_right.run(Raspi_MotorHAT.FORWARD)


    if (data.data):
        if (data.data[0]==4):
            rospy.loginfo("I detcet  %s is TURN LEFT", data.data[0])
            Motor_left.setSpeed(80)
            Motor_right.setSpeed(80)
            Motor_left.run(Raspi_MotorHAT.BACKWARD)
            Motor_right.run(Raspi_MotorHAT.FORWARD)
            rospy.sleep(2.)
            Motor_left.run(Raspi_MotorHAT.FORWARD)
            Motor_right.run(Raspi_MotorHAT.FORWARD)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("objects", Float32MultiArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    print("find 2d object START!!")
    listener()
