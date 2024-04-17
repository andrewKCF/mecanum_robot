#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image # subscribe image
from std_msgs.msg import Float64 # setpoint state # contro_effort
from std_msgs.msg import Bool # setpoint state
import numpy as np
import cv2

midx=0
image_height=0
image_width=0

def callback(data):
    # define picture to_down' coefficient of ratio
    global bridge
    global midx,image_height,image_width

    img = bridge.imgmsg_to_cv2(data, "bgr8")

    #define kernel size  
    kernel = np.ones((7,7),np.uint8)
    # convert to hsv colorspace 
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower bound and upper bound
    lower_bound = np.array([  0, 103, 108])     
    upper_bound = np.array([179, 255, 255])
    # find the colors within the boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    # Remove unnecessary noise from mask
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # Segment only the detected region
    segmented_img = cv2.bitwise_and(img, img, mask=mask)
    # Find contours from the mask
    _,contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the index of the largest contour
    areas=[cv2.contourArea(c) for c in contours]
    if not areas:
        midx=image_width/2
        cv2.imshow("color_track",img)
        cv2.waitKey(1)
        return
       

    max_index=np.argmax(areas)
    cnt=contours[max_index]
    # Draw boundary box
    x,y,w,h=cv2.boundingRect(cnt)
    cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
    midx=x+w/2
    midy=y+h/2
    cv2.circle(img,(midx,midy),2,(0,0,255),4)
    
    
    cv2.imshow("color_track" , img)
    cv2.waitKey(1)

def color_tracker():
    rospy.init_node('color_tracker', anonymous=True)
    # make a video_object and init the video object
    global bridge,midx
    bridge = CvBridge()
    rospy.Subscriber('usb_cam/image_raw', Image, callback)
    rate = rospy.Rate(10) # 10hz
    pub_state = rospy.Publisher('state', Float64, queue_size=10)
    pub_setpoint = rospy.Publisher('setpoint', Float64, queue_size=10)
    pub_enable = rospy.Publisher('pid_enable', Bool, queue_size=10)

    while not rospy.is_shutdown():
        pub_enable.publish(True)
        pub_setpoint.publish(image_width/2)
        print(midx)
        pub_state.publish(midx)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    image_height=rospy.get_param('/usb_cam/image_height')
    image_width=rospy.get_param('/usb_cam/image_width')
    color_tracker()
