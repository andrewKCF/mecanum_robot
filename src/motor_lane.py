#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 #contro_effort
from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
mh = Raspi_MotorHAT(addr=0x60)
Motor_left=mh.getMotor(1)
Motor_right=mh.getMotor(2)




def callback(data):
    speed=80
    pwm=abs(int(data.data)+speed)
    Motor_left.run(Raspi_MotorHAT.FORWARD)
    Motor_right.run(Raspi_MotorHAT.FORWARD)
    #print("contro_effort %", data.data)
    str="pwm (fix)left:%3d"%speed +", right:%3d"%pwm
    print(str)
    if (data.data)==0:
        Motor_left.setSpeed(0)
        Motor_right.setSpeed(0)
    else:
        Motor_left.setSpeed(speed)
        Motor_right.setSpeed(pwm)


    
def listener():
    rospy.init_node('motor_lane', anonymous=True)
    rospy.Subscriber("control_effort", Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
