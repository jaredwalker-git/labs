#! /usr/bin/env python3

import numpy as np
import rospy
from pid import PID
from duckietown_msgs.msg import WheelsCmdStamped, AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Transform
import time


class Lab4Part1():

    def __init__(self):
        rospy.init_node("lab4_part1", anonymous=True)
        rospy.Subscriber("/duckiekong/apriltag_detector_node/detections", AprilTagDetectionArray, self.callback)
        self.pub = rospy.Publisher('/duckiekong/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 0)
        
        self.pid_angular = PID(0.4,0.0075,0.05)

        ######self.pid_angular = PID(0.1,0.001,0.5)
        self.pid_forward = PID(1.2,0.005,0.1)
        self.wheel_cmd = WheelsCmdStamped()

        ## IN POSITION, -X = Left, +X = Right, Z = Distance

        #while(not(rospy.has_param("controller_ready"))):
        #    sleep(0.01)
        #rospy.set_param("controller_ready", "true")

    def callback(self, msg):
        if len(msg.detections) > 0:
            msg = msg.detections[0]
            #error between april tag position in camera frame and ideal position. + error = left, - error = right
            side_error = 0 - msg.transform.translation.x
            front_error = msg.transform.translation.z - 0.11

            angular = self.angle(side_error) * 2
            forward = self.forward(front_error)
            rospy.logerr(forward)
            total_right = forward+angular
            total_left = -angular+forward
            rospy.logwarn("left before" + str(total_left))
            rospy.logwarn("right before" + str(total_right))
            if(total_right > 0.5):
                total_right = 0.5
            elif(total_right < -0.5):
                total_right = -0.5

            if(total_left > 0.5):
                total_left = 0.5
            elif(total_left < -0.5):
                total_left = -0.5

            self.wheel_cmd.vel_right =  total_right
            self.wheel_cmd.vel_left = total_left

            rospy.logwarn("left after" + str(total_left))
            rospy.logwarn("right after" + str(total_right))
            self.pub.publish(self.wheel_cmd)


    def angle(self, side_error):
        angular = self.pid_angular.get_control(rospy.get_time(), side_error)

        #works with 0.3
        if (angular > 0.3):
            angular = 0.3
        elif(angular < -0.3):
            angular = -0.3

        #self.wheel_cmd.vel_right =  angular
        #self.wheel_cmd.vel_left = -(angular)


        #rospy.logerr(side_error)
        #self.pub.publish(self.wheel_cmd)
        return(angular)
        

    def forward(self, front_error):

        #self.wheel_cmd.vel_left = 0.00
        #self.wheel_cmd.vel_right = 0.00

        forward = self.pid_forward.get_control(rospy.get_time(), front_error)

        if (forward > 0.3):
            forward = 0.3
        elif (forward < -0.3):
            forward = -0.3

        #self.wheel_cmd.vel_left = forward
        #self.wheel_cmd.vel_right = forward
        #rospy.logwarn(forward)
        #self.pub.publish(self.wheel_cmd)
        return(forward)
    

if  __name__ == '__main__':

    node = Lab4Part1()
    rospy.spin()
