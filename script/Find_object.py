#!/usr/bin/env python2.7
# license removed 

# # The ROS staff
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import geometry_msgs
from geometry_msgs import Point
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError

import cv2 as cv
import numpy as np

def Find_object():
    
    #rospy.init_node('talker',anonymous = True)
    rospy.init_node('Find_object',anonymous = True)
    rate = rospy.Rate(10)
    rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,callback,queue_size=1,buff_size=65536)
    

def callback(data):
    try:
        cv_image = cv.bridge.CompressedImage_to_cv2(data,'passthrough')
        point = find_object(cv_image)
        pub.publish(point)
        rate.sleep()
    except CvBridgeError:
        rospy.logerr('CvBridge Error')
        rospy.spin()

    
    
    return cv_image



def find_object(img):
            Compare_rgb = img# read the file 
            #W = Compare_rgb.shape[1]
            #err_allow = err_allow*W # the err_allowed from middle
            #Middle = W//2
            target_template = cv.imread('object_target.jpg')
            threshold = 0.6
            
            
            for scale in range(245,900,10):
                
                frame = Compare_rgb.copy()
                ratio = int(frame.shape[1]/frame.shape[0])
                ratio=frame.shape[1]/frame.shape[0]
                frame=cv.resize(frame,(frame.shape[1]+round(ratio*scale),frame.shape[0]+scale))
                res=cv.matchTemplate(frame,target_template,cv.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
                
                if max_val < threshold:
                    continue
                else:
                    top_left = max_loc
                    middle_point = (top_left[0]+target_template.shape[1]//2,top_left[1]+target_template.shape[0]//2)
                    return middle_point
                    # break

            # if middle_point[0] < Middle - err_allow:
            #     print('R')
            #     return 'R'
            # if middle_point[0] > Middle + err_allow:
            #     print('L')
            #     return 'L'
            # else:
            #     print('S')
                # return 'S'



if __name__ == '__main__':
    pub = rospy.Publisher('obj_coord',Point, queue_size=10)
    
    try:
        Find_object()
    except rospy.ROSInterruptException:
        rospy.spin()
