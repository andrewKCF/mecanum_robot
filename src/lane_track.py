#!/usr/bin/env python
#!coding=utf-8
import numpy as np
import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image # subscribe image
from std_msgs.msg import Float64 # setpoint state # contro_effort
from std_msgs.msg import Bool # setpoint state

state=0
image_height=0
image_width=0

def callback(data):
    # define picture to_down' coefficient of ratio
    global bridge
    global state,image_height,image_width
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    blur = cv2.GaussianBlur(img,(5,5),0)
    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)

    low_lane = np.array([  4, 144, 134])
    high_lane = np.array([ 79, 199, 177])
    mask = cv2.inRange(hsv,low_lane,high_lane)
    kernel = np.ones((1,1), np.uint8)
    mask = cv2.erode(mask, kernel, iterations = 1)

    canny = cv2.Canny(mask,50,150)

    #lines = cv2.HoughLinesP(canny,1,np.pi/180,50,maxLineGap=50,minLineLength=20)
    lines = cv2.HoughLinesP(canny,1,np.pi/120,10,maxLineGap=5,minLineLength=15)
    #if not ('lines' in locals()):
    if lines is None:
        state=image_width
        cv2.imshow("lane_track" , mask)
        cv2.waitKey(1)
        return
    #print(lines.shape)
    for line in lines:
        x1,y1,x2,y2 = line[0]
        #print('x1:{:.2f}  y1:{:.2f}  x2:{:.2f}  y2:{:.2f}'.format(x1,y1,x2,y2))
        dy=y2-y1
        dx=x2-x1
    
        if (dy/dx) < 0:
            break

        fx=round(abs((image_height-y1)*dx/dy+x1))
        fx=int(fx)
        cv2.circle(img,(image_width/2,image_height), 10, (0, 255, 255), 3)
        if (x1>image_width/2):
            state=fx
            cv2.line(img,(x1,y1),(x2,y2),(255,0,0),3)
            cv2.line(img,(x1,y1),(x2,y2),(255,0,0),3)
            cv2.circle(img,(fx,image_height), 10, (0, 255, 0), 3)
            #print('state:{:.2f}   dy/dx:{:.2f}'.format(state,dy/dx))
            break
    
    cv2.imshow("lane_track", img)
    cv2.waitKey(1)

def lane_track():
    rospy.init_node('lane_track', anonymous=True)
    # make a video_object and init the video object
    global bridge,state
    bridge = CvBridge()
    rospy.Subscriber('usb_cam/image_raw', Image, callback)
    rate = rospy.Rate(10) # 10hz
    pub_state = rospy.Publisher('state', Float64, queue_size=10)
    pub_setpoint = rospy.Publisher('setpoint', Float64, queue_size=10)
    pub_enable = rospy.Publisher('pid_enable', Bool, queue_size=10)

    while not rospy.is_shutdown():
        pub_enable.publish(True)
        pub_setpoint.publish(image_width)
        pub_state.publish(state)
        str="setpoint %3.2f"%image_width + ",  state %3.2f"%state
        rospy.loginfo(str)
        #print("state:%5f",state)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    image_height=rospy.get_param('/usb_cam/image_height')
    image_width=rospy.get_param('/usb_cam/image_width')
    lane_track()

