#! /usr/bin/env python3

import numpy as np
import rospy
from pid import PID
from duckietown_msgs.msg import LanePose, Twist2DStamped



class Lab4Part2():
    def __init__(self):
        self.wheel_cmd_pub = rospy.Publisher('/duckiekong/lane_controller_node/car_cmd', Twist2DStamped, queue_size = None)
        self.pose_sub = rospy.Subscriber('/duckiekong/lane_filter_node/lane_pose', LanePose, self.command_cb)
        self.controller_d = PID(10.0, 0.2, 0)
        self.controller_phi = PID(4.5, 0, 0)
        self.command = Twist2DStamped()
        
    def command_cb(self, posemsg):
        d_error = 0 - posemsg.d  #negative is right of lane - positive is left of lane for both - range abs(0-0.3ish)
        phi_error = 0 - posemsg.phi   
        total_err = phi_error + d_error
        control_phi = 0 
        control_d = 0
        
        if abs(d_error) > 0.01:
            control_d = self.controller_d.get_control(rospy.get_time(), d_error)
        
        if abs(phi_error) > 0.1:
            control_phi = self.controller_phi.get_control(rospy.get_time(),phi_error)
        
        control = control_d + control_phi
        self.command.v = 0.2 #slow down when error is present - .2 was base speed for duckietown
        #positive omega to turn left, around 1-2
        self.command.omega = control 
        self.wheel_cmd_pub.publish(self.command)
            
        rospy.logwarn(control)
        
        
if __name__ == '__main__':
    rospy.init_node('lab4prt2')
    agent = Lab4Part2()
    rospy.spin()
