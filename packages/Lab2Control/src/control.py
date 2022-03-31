#!/usr/bin/env python3

import rospy
import time
from duckietown_msgs.msg import WheelsCmdStamped

#speeds initialized based on joystick commands
class RosAgent:
    def __init__(self):
        self.publisher = rospy.Publisher('wheels_driver_node/wheels_cmd', WheelsCmdStamped)
        #this has type duckietown_msgs/FSMState, probably should be change to LANE_FOLLOWING
        self.subscriber = rospy.Subscriber('fsm_node/mode', WheelsCmdStamped, self.callback)
    
    def callback(self, msg): #set for half of the square currently

        #uses the fsm_state to perform our movement when the lane following demo is started
        if(msg.state == "LANE_FOLLOWING"):
            self.wheel_cmd = WheelsCmdStamped()
            self.forward()
            self.wait()
            self.turn()
            self.forward()
            self.wait()

    def forward(self): #function to move 1m
        self.wheel_cmd.vel_left = 0.47
        self.wheel_cmd.vel_right = 0.47
        self.publisher.Publish(self.wheel_cmd)
        time.sleep(2) #time initialized at 3s for first attempt
        self.wheel_cmd.vel_left = 0.0
        self.wheel_cmd.vel_right = 0.0
        self.publisher.Publish(self.wheel_cmd)
        
    def turn(self): #function to turn 90 degrees right
        self.wheel_cmd.vel_left = 0.23
        self.wheel_cmd.vel_right = -0.23
        self.publisher.Publish(self.wheel_cmd) #turn half speed for accuracy
        time.sleep(1) #1s is first initialization for turn time
        self.wheel_cmd.vel_left = 0.0
        self.wheel_cmd.vel_right = 0.0
        self.publisher.Publish(self.wheel_cmd)

    def wait(self): #function to wait 5s
    	time.sleep(5)


if __name__ == '__main__':
    rospy.init_node('onemetersquare')
    rospy.spin()
    agent = RosAgent()

