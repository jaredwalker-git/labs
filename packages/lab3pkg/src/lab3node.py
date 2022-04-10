#!/usr/bin/env python3

import rospy
import cv2
import os.path
from os import path
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class RosAgent:
    def __init__(self):
        self.camera_sub = rospy.Subscriber('/camera_node/image/compressed', CompressedImage, queue_size = 1, buff_size = 2**24,  self.filter_cb)
        self.whitepub = rospy.Publisher('/lab3_white_line', Image, queue_size = 1)
        self.yellowpub = rospy.Publisher('/lab3_yellow_line', Image, queue_size = 1) 
        self.cannykernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
        
    def filter_cb(self, img):
        cvimg = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
        cvimg_resize = cv2.resize(cvimg, (160,120), interpolation=cv2.INTER_NEAREST)
        cvimg_resize = cvimg_resize[40:,:]
        hsvimg = cv2.cvtColor(cvimg_resize, cv2.COLOR_BGR2HSV)
        
        white_filteredhsv = cv2.inRange(hsvimg, (0, 0, 100), (180, 33, 255))
        yellow_filteredhsv = cv2.inRange(hsvimg, (29, 120, 100), (31, 255, 255))
        
        white_filteredhsv = cv2.erode(white_filteredhsv, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4)))
        white_filteredhsv = cv2.dilate(white_filteredhsv, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4)))
        yellow_filteredhsv = cv2.erode(yellow_filteredhsv, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)))
        yellow_filteredhsv = cv2.dilate(yellow_filteredhsv, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4)))
        
        edge_img = cv2.Canny(cvimg_resize, 175, 220) 
        edge_img = cv2.dilate(edge_image, self.kernel)
        
        def create_houghimgmsg(img_overlay):
            houghlines = []
            img_overlay = np.array(img_overlay)
            houghlines = cv2.HoughLinesP(img_overlay, 1, 3.1415/180, 50)
            white_line_image = self.output_lines(self.cropped_img, houghlines)
            line_imgmsg = self.bridge.cv2_to_imgmsg(white_line_image, "bgr8")
            return line_imgmsg
        
        img_overlay_white = []
        img_overlay_white = np.array(img_overlay_white)
        img_overlay_white = cv2.bitwise_and(np.array(white_filteredhsv), np.array(edge_img)).astype('uint8')
        img_overlay_yellow = []
        img_overlay_yellow = np.array(img_overlay_yellow)
        img_overlay_yellow = cv2.bitwise_and(np.array(yellow_filteredhsv), np.array(edge_img)).astype('uint8')
        line_msg_white = create_houghimgmsg(img_overlay_white)
        line_msg_yellow = create_houghimgmsg(img_overlay_yellow)
                    
        self.whitepub.publish(line_msg_white)
        self.yellowpub.publish(line_msg_yellow)
        
if __name__ == "__main__":
    rospy.init_node("lab3node")
    rosagent = RosAgent()
    rospy.spin()
        
        
        
