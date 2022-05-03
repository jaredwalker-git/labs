#!/usr/bin/env python3

import rospy
import cv2
import os.path
from os import path
import numpy as np
from duckietown_msgs.msg import SegmentList
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class RosAgent:
    def __init__(self):
        self.camera_sub = rospy.Subscriber('/duckiekong/camera_node/image/compressed', CompressedImage, self.filter_cb, queue_size = 1, buff_size = 2**24)
        self.segmentpub = rospy.Publisher('/duckiekong/line_detector_node/segment_list', SegmentList)
        self.cannykernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
        self.cvbridge = CvBridge()
        #each line segment is own segment within list
        self.segment
        
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
        lines_white = self.create_houghlines(img_overlay_white)
        lines_yellow = self.create_houghlines(img_overlay_yellow)
        self.white_lines.pixels_normalized = lines_white
        self.yellow_lines.pixels_normalized = lines_yellow
        self.whitepub.publish(lines_white)
        self.yellowpub.publish(lines_yellow)

    def create_houghlines(self, img_overlay):
        houghlines = []
        img_overlay = np.array(img_overlay)
        img_size = (160,120)
        offset = 40
        self.cvimg_resize = cv2.resize(img_overlay, img_size, interpolation=cv2.INTER_NEAREST)
        self.cvimg_resize = self.cvimg_resize[offset:,:]
        houghlines = cv2.HoughLinesP(self.cvimg_resize, 1, 3.1415/180, 50)
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / img_size[0], 1. / img_size[1], 1. / img_size[0], 1. / img_size[1]])
        normalized = (line + arr_cutoff) * arr_ratio
        return normalized
        

if __name__ == "__main__":
    rospy.init_node("lab3node")
    rosagent = RosAgent()
    rospy.spin()
        
        
        
