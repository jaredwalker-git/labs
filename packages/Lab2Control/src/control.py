#!/usr/bin/env python3

import rospy
import time
from duckietown_msgs.msg import WheelsCmdStamped, FSMState
from odometry_hw.msg import Pose2D

#speeds initialized based on joystick commands
class RosAgent:
    def __init__(self):
        self.publisher = rospy.Publisher('wheels_driver_node/wheels_cmd', WheelsCmdStamped)
        self.subscriber = rospy.Subscriber('fsm_node/mode', FSMState, self.callback)
        self.posesub = rospy.Subscriber('/pose', Pose2D, self.print_cb)
        self.wheel_cmd = WheelsCmdStamped()
        self.firstpose = None
        self.lastpose_msg = None
    
    def print_cb(self, msg):
        self.lastpose_msg = msg
    
        if self.firstpose == None:
            self.firstpose = self.lastpose_msg
        
        
    def forward(self): #function to move 1m
        r = rospy.Rate(10)
        start_time = rospy.get_time()
        
        while ((rospy.get_time() - start_time) < 3.15):
            self.wheel_cmd.vel_left = 0.49
            self.wheel_cmd.vel_right = 0.47
            self.publisher.publish(self.wheel_cmd)
            r.sleep()
            
        self.wheel_cmd.vel_left = 0.0
        self.wheel_cmd.vel_right = 0.0
        self.publisher.publish(self.wheel_cmd)
        
    def turn(self): #function to turn 90 degrees right

        r = rospy.Rate(10)
        start_time = rospy.get_time()
        
        while ((rospy.get_time() - start_time) < .8):
            self.wheel_cmd.vel_left = 0.26
            self.wheel_cmd.vel_right = -0.26
            self.publisher.publish(self.wheel_cmd)
            r.sleep()  
      
        self.wheel_cmd.vel_left = 0.0
        self.wheel_cmd.vel_right = 0.0
        self.publisher.publish(self.wheel_cmd)

    	
    def print_odom(self, lastpose, firstpose):
        truepose = Pose2D()
        truepose.x = lastpose.x - firstpose.x
        truepose.y = lastpose.y - firstpose.y
        rospy.logwarn(truepose)
    
    def callback(self, msg): #set for half of the square currently

        #uses the fsm_state to perform our movement when the lane following demo is started
        if(msg.state == 'LANE_FOLLOWING'):
            self.print_odom(self.lastpose_msg, self.firstpose)
            
            time.sleep(2)
            self.forward()
            time.sleep(2)
            self.turn()
            self.print_odom(self.lastpose_msg, self.firstpose)
            
            time.sleep(2)
            self.forward()
            time.sleep(2)
            self.turn()
            self.print_odom(self.lastpose_msg, self.firstpose)
            
            time.sleep(2)
            self.forward()     
            time.sleep(2)
            self.turn()
            self.print_odom(self.lastpose_msg, self.firstpose)
            
            
        if(msg.state == 'NORMAL_JOYSTICK_CONTROL'):
            self.wheel_cmd.vel_left = 0.0
            self.wheel_cmd.vel_right = 0.0
            self.publisher.publish(self.wheel_cmd)
            
            
if __name__ == '__main__':
    rospy.init_node('onemetersquare')
    agent = RosAgent()
    rospy.spin()
    

