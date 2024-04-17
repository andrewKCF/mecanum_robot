#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
from datetime import datetime
import math
# create a default object, no changes to I2C address or frequency
mh = Raspi_MotorHAT(addr=0x60)

Motor_left=mh.getMotor(1)
Motor_right=mh.getMotor(2)



def callback(data):
        print('stick left LR 0:%s'%data.axes[0])
        print('stick left UD 1:%s'%data.axes[1])
        print('stick right LR 2:%s'%data.axes[2])                
        print('stick right UD 3:%s'%data.axes[3])                
        print('R2:%s'%data.axes[4])
        print('L2:%s'%data.axes[5])
        print('cross key LR:{:.50f}'.format(data.axes[6]))
        print('cross key UD:%s'%data.axes[7])
        

        print('A:%s'%data.buttons[0])
        print('B:%s'%data.buttons[1])
        #print('2:%s'%data.buttons[2])
        print('X:%s'%data.buttons[3])
        print('Y:%s'%data.buttons[4])
        #print('5:%s'%data.buttons[5])
        print('L1:%s'%data.buttons[6])
        print('R1:%s'%data.buttons[7])
        print('L2:%s'%data.buttons[8])
        print('R2:%s'%data.buttons[9])
        print('SELECT:%s'%data.buttons[10])
        print('START:%s'%data.buttons[11])
        #print('12:%s'%data.buttons[12])
        print('stick left bu:%s'%data.buttons[13])
        print('stick right bu:%s'%data.buttons[14])
        now=datetime.now()
        current_time=now.strftime("%H:%M:%S")
        print("current time:",current_time)
        

        if (data.axes[6]!=0 or data.axes[7]!=0):
                if (data.axes[7]>0):
                        print("forward")
                        Motor_left.setSpeed(100)
                        Motor_right.setSpeed(100)
                        Motor_left.run(Raspi_MotorHAT.FORWARD)
                        Motor_right.run(Raspi_MotorHAT.FORWARD)
                elif(data.axes[7]<0):
                        print("backward")
                        Motor_left.setSpeed(100)
                        Motor_right.setSpeed(100)
                        Motor_left.run(Raspi_MotorHAT.BACKWARD)
                        Motor_right.run(Raspi_MotorHAT.BACKWARD)
                if (data.axes[6]>0):
                        print("left")
                        Motor_left.setSpeed(100)
                        Motor_right.setSpeed(100)
                        Motor_left.run(Raspi_MotorHAT.BACKWARD)
                        Motor_right.run(Raspi_MotorHAT.FORWARD)
                elif(data.axes[6]<0):
                        print("right")
                        Motor_left.setSpeed(100)
                        Motor_right.setSpeed(100)
                        Motor_left.run(Raspi_MotorHAT.FORWARD)
                        Motor_right.run(Raspi_MotorHAT.BACKWARD)
        else:
                Motor_left.setSpeed(0)
                Motor_right.setSpeed(0)

# Intializes everything
def start():
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, callback)
	# starts the node
	rospy.init_node('joy2car')
	rospy.spin()

if __name__ == '__main__':
        print("joy2car START")
	start()
