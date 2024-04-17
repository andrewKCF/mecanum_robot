#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
joystr=""

def callback(data):
        global joystr
        '''
        joystr=""
        for i in range(15):
            joystr=joystr+str(data.buttons[i])
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
       '''
        joystr=" "
        if (data.buttons[0]):
            joystr="A"
        if (data.buttons[1]):
            joystr="B"
        if (data.buttons[3]):
            joystr="X"
        if (data.buttons[4]):
            joystr="Y"

        if (data.buttons[6]):
            joystr="L1"
        if (data.buttons[7]):
            joystr="R1"

        if (data.buttons[8]):
            joystr="L2"
        if (data.buttons[9]):
            joystr="R2"

        if (data.axes[6]>0):
            joystr="L"
        if (data.axes[6]<0):
            joystr="R"
        if (data.axes[7]>0):
            joystr="U"
        if (data.axes[7]<0):
            joystr="D"

      
def controller():
    global joystr
    #Declare node name
    rospy.init_node('joy2str', anonymous=True)
    #Create a Subscriber. Load the topic.
    sub = rospy.Subscriber("joy", Joy, callback)
    #Create Publisher('Topic name',Mold,size)
    pub = rospy.Publisher('joy2str', String, queue_size=1)
    #Loop period.
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        #Publish data
        pub.publish(joystr)
        rospy.loginfo(joystr)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInitException:
        pass
