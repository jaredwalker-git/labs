#!/usr/bin/env python3

import rospy
import cv2
import os.path
from os import path
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class RosAgent:
    def __init__(self):
        self.camera_sub = rospy.Subscriber('/duckiekong/camera_node/image/compressed', CompressedImage, self.filter_cb, queue_size = 1, buff_size = 2**24)
        self.whitepub = rospy.Publisher('/lab3_white_line', Image, queue_size = 1)
        self.yellowpub = rospy.Publisher('/lab3_yellow_line', Image, queue_size = 1) 
        
        self.cannykernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
        self.cvbridge = CvBridge()
        
    def filter_cb(self, img):
        self.cvimg = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")

        hsvimg = cv2.cvtColor(self.cvimg, cv2.COLOR_BGR2HSV)
        
        white_filteredhsv = cv2.inRange(hsvimg, (0, 0, 100), (180, 33, 255))
        yellow_filteredhsv = cv2.inRange(hsvimg, (10, 80, 100), (40, 255, 255))
        
        white_filteredhsv = cv2.erode(white_filteredhsv, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4)))
        white_filteredhsv = cv2.dilate(white_filteredhsv, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4)))
        yellow_filteredhsv = cv2.erode(yellow_filteredhsv, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)))
        yellow_filteredhsv = cv2.dilate(yellow_filteredhsv, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4)))
        
        edge_img = cv2.Canny(self.cvimg, 175, 220) 
        edge_img = cv2.dilate(edge_img, self.cannykernel)
        
        img_overlay_white = []
        img_overlay_white = np.array(img_overlay_white)
        img_overlay_white = cv2.bitwise_and(np.array(white_filteredhsv), np.array(edge_img)).astype('uint8')
        img_overlay_yellow = []
        img_overlay_yellow = np.array(img_overlay_yellow)
        img_overlay_yellow = cv2.bitwise_and(np.array(yellow_filteredhsv), np.array(edge_img)).astype('uint8')
        line_msg_white = self.create_houghimgmsg(img_overlay_white)
        line_msg_yellow = self.create_houghimgmsg(img_overlay_yellow)
        
                   
        self.whitepub.publish(line_msg_white)
        self.yellowpub.publish(line_msg_yellow)

    def create_houghimgmsg(self, img_overlay):
        houghlines = []
        img_overlay = np.array(img_overlay)
        houghlines = cv2.HoughLinesP(img_overlay, 1, 3.1415/180, 50)
        line_image = self.output_lines(self.cvimg, houghlines)
        self.cvimg_resize = cv2.resize(line_image, (160,120), interpolation=cv2.INTER_NEAREST)
        self.cvimg_resize = self.cvimg_resize[40:,:]
        line_imgmsg = self.cvbridge.cv2_to_imgmsg(self.cvimg_resize, "bgr8")
        return line_imgmsg
        
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
        
if __name__ == "__main__":
    rospy.init_node("lab3node")
    rosagent = RosAgent()
    rospy.spin()
        
        
        
