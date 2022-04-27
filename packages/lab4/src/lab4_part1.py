#! /usr/bin/env python3

import numpy as np
import rospy
from pid import PID
from duckietown_msgs.msg import WheelsCmdStamped, AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Transform


class Lab4Part1():

    def __init__(self):
        rospy.init_node("lab4_part1", anonymous=True)
        rospy.Subscriber("/duckiekong/apriltag_detector_node/detections", AprilTagDetectionArray, self.callback)
        self.pub = rospy.Publisher('/duckiekong/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 0)
        
        self.pid_angular = PID(1,0.0,0.)

        ##self.pid_angular = PID(0.1,0.001,0.5)
        #self.pid_forward = PID(1,0.0,0.0)
        self.wheel_cmd = WheelsCmdStamped()

        ## IN POSITION, -X = Left, +X = Right, Z = Distance

        #while(not(rospy.has_param("controller_ready"))):
        #    sleep(0.01)
        #rospy.set_param("controller_ready", "true")

    def callback(self, msg):
        if len(msg.detections) > 0:
            msg = msg.detections[0]
            #if abs(msg.transform.translation.x) > 0.02:
            self.angle(msg)
            #else:
                #self.wheel_cmd.vel_left = 0.00
                #self.wheel_cmd.vel_right = 0.00

                #self.pub.publish(self.wheel_cmd)

        #elif (msg.transform.translation.z > dist):
            #self.forward(msg)

    def angle(self, msg):
        #error between april tag position in camera frame and ideal position. + error = left, - error = right
        side_error = 0 - msg.transform.translation.x

        ang = 1

        self.wheel_cmd.vel_left = 0.00
        self.wheel_cmd.vel_right = 0.00

        angular = self.pid_angular.get_control(rospy.get_time(), side_error)
        self.wheel_cmd.vel_right = ang * angular
        self.wheel_cmd.vel_left = ang * -(angular)
        rospy.logerr(angular)
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
