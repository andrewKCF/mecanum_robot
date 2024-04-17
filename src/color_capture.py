import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import cv2
import numpy as np
from cspaceSliders import FilterWindow


'''
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ret, img = cap.read()
    cv2.imshow('Video', img)
    
    key=cv2.waitKey(1)
    if key==ord('q'):
        cv2.imwrite('photo.jpg',img)
        break

cap.release()
cv2.destroyAllWindows()        



window = FilterWindow('Filter Window', img)
window.show(verbose=True)

colorspace = window.colorspace
lowerb, upperb = window.bounds
mask = window.mask
applied_mask = window.applied_mask

print('Displaying the image with applied mask filtered in', colorspace,
      '\nwith lower bound', lowerb, 'and upper bound', upperb)
cv2.imshow('Applied Mask', applied_mask)
cv2.waitKey()
'''

def callback(data):
    # define picture to_down' coefficient of ratio
    global bridge
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("frame" , img)
    key=cv2.waitKey(1)
    if key==ord('q'):
        cv2.imwrite('photo.jpg',img)
        window = FilterWindow('Filter Window', img)
        window.show(verbose=True)
        colorspace = window.colorspace
        lowerb, upperb = window.bounds
        mask = window.mask
        applied_mask = window.applied_mask
        print('Displaying the image with applied mask filtered in', colorspace,
          '\nwith lower bound', lowerb, 'and upper bound', upperb)
        cv2.imshow('Applied Mask', applied_mask)        
    

def captureColor():
    rospy.init_node('captureColor', anonymous=True)
    # make a video_object and init the video object
    global bridge
    bridge = CvBridge()
    rospy.Subscriber('usb_cam/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    captureColor()

