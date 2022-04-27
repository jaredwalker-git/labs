#! /usr/bin/env python3

import numpy as np
import rospy
from pid import PID
from duckietown_msgs.msg import WheelsCmdStamped


class Lab4Part1():

    def __init__(self):
        rospy.init_node("lab4_part1", anonymous=True)
        rospy.Subscriber("/tag_detections", , self.callback)
        self.publisher = rospy.Publisher('wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 0)
        
        self.pid_angular = PID(0.25,0.02,0.7)
        self.pid_forward = PID(0.25,0.02,0.7)
        self.wheel_cmd = WheelsCmdStamped()
        #while(not(rospy.has_param("controller_ready"))):
        #    sleep(0.01)
        #rospy.set_param("controller_ready", "true")

    def callback(self, msg):

        if #angle is out of range
        self.angle(msg)

        else if #distance is too large
        self.forward(msg)

    def angle(self, msg):
        ang = 0.2

        self.wheel_cmd.vel_left = 0.00
        self.wheel_cmd.vel_right = 0.00

        angular = self.pid_angular.get_control(rospy.get_time(), msg.data())
        self.wheel_cmd.vel_left = ang * angular
        self.wheel_cmd.vel_right = ang * -(angular)
        self.pub.publish(self.wheel_cmd)

    def forward(self, msg):
        self.wheel_cmd.vel_left = 0.00
        self.wheel_cmd.vel_right = 0.00

        forward = self.pid_forward.get_control(rospy.get_time(), msg.data())
        self.wheel_cmd.vel_left = forward
        self.wheel_cmd.vel_right = forward

        self.pub.publish(self.wheel_cmd)
    

if  __name__ == '__main__':

    node = Lab4Part1()
    rospy.spin()
