#!/usr/bin/env python3

import rospy
import time
from duckietown_msgs.msg import WheelCmdStamped

#speeds initialized based on joystick commands
class RosAgent:
    def __init__(self):
        self.publisher = rospy.Publisher('lane_controller_node/car_cmd', WheelsCmdStamped)
        self.subscriber = rospy.Subscriber('fsm_node/mode', WheelsCmdStamped, self.callback)
    
    def callback(self): #set for half of the square currently
        forward()
        wait()
        turn()
        forward()
        wait()

    def forward(self): #function to move 1m
        self.publisher.Publish('{vel_left: 0.47,vel_right: 0.47}')
        time.sleep(3) #time initialized at 3s for first attempt
        self.publisher.Publish('{vel_left: 0.0,vel_right: 0.0}')
        
    def turn(self): #function to turn 90 degrees right
        self.publisher.Publish('{vel_left: 0.23,vel_right: 0.0}') #turn half speed for accuracy
        time.sleep(1) #1s is first initialization for turn time
        self.publisher.Publish('{vel_left: 0.0,vel_right: 0.0}')

    def wait(self): #function to wait 5s
    	time.sleep(5)


if __name__ == '__main__':
    rospy.init_node('onemetersquare')
    rospy.spin()
    agent = RosAgent()

