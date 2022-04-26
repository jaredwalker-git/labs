#!/usr/bin/env python3

import numpy as np
import rospy
from odometry_hw.msg import Pose2D
from duckietown_msgs.msg import WheelEncoderStamped

class currentCoords:

    def __init__(self):
        rospy.Subscriber('/duckiekong/right_wheel_encoder_node/tick', WheelEncoderStamped, self.storepose_right)
        rospy.Subscriber('/duckiekong/left_wheel_encoder_node/tick', WheelEncoderStamped, self.storepose_left)
        self.publisher = rospy.Publisher('/pose', Pose2D, queue_size = 10)
        self.wheeldist = 0.05 #dist from center robot to wheel
        self.left_wheel_tick = 0 
        self.right_wheel_tick = 0
        self.right_wheel_tick_last = 0
        self.left_wheel_tick_last = 0
        self.currentx = 0
        self.currenty = 0
        self.currenttheta = 0

    
    def storepose_right(self, right_ticks):
        self.right_wheel_tick = right_ticks.data
        #689.5,677 / 669.5, 673.50 -> avg 677 ticks/meter
                
    def storepose_left(self, left_ticks):
        self.left_wheel_tick = left_ticks.data

if __name__ == "__main__":
    rospy.init_node('lab2prt2')
    pubmsg = Pose2D()
    poseFinder = currentCoords()
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        #find delta tick
        right_tick_change = poseFinder.right_wheel_tick - poseFinder.right_wheel_tick_last
        left_tick_change = poseFinder.left_wheel_tick - poseFinder.left_wheel_tick_last
        #set last ticks
        poseFinder.right_wheel_tick_last = poseFinder.right_wheel_tick
        poseFinder.left_wheel_tick_last = poseFinder.left_wheel_tick
        #ticks to meters
        right_dist_change = right_tick_change/677
        left_dist_change = left_tick_change/677
        #find delta S and anglechange
        totaldist = (right_dist_change + left_dist_change)/2
        anglechange = (right_dist_change - left_dist_change)/(2 * poseFinder.wheeldist)
        changex = totaldist * np.cos(poseFinder.currenttheta + anglechange/2)
        changey = totaldist * np.sin(poseFinder.currenttheta + anglechange/2)
        poseFinder.currentx = poseFinder.currentx + changex
        poseFinder.currenty = poseFinder.currenty + changey
        poseFinder.currenttheta = poseFinder.currenttheta + anglechange
        pubmsg.x = poseFinder.currentx
        pubmsg.y = poseFinder.currenty
        pubmsg.theta = poseFinder.currenttheta
        poseFinder.publisher.publish(pubmsg)
        r.sleep()



