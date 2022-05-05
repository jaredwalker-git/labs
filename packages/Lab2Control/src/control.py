#!/usr/bin/env python3

import rospy
import time
from duckietown_msgs.msg import WheelsCmdStamped, FSMState

#speeds initialized based on joystick commands
class RosAgent:
    def __init__(self):
        self.publisher = rospy.Publisher('wheels_driver_node/wheels_cmd', WheelsCmdStamped)
        self.subscriber = rospy.Subscriber('fsm_node/mode', FSMState, self.callback)
        self.wheel_cmd = WheelsCmdStamped()
    
    def forward(self): #function to move 1m

        self.wheel_cmd.vel_left = 0.5
        self.wheel_cmd.vel_right = 0.5
        self.publisher.publish(self.wheel_cmd)
        time.sleep(0.15)
        self.publisher.publish(self.wheel_cmd)
        time.sleep(3) #time initialized at 3s for first attempt
        self.wheel_cmd.vel_left = 0.0
        self.wheel_cmd.vel_right = 0.0
        self.publisher.publish(self.wheel_cmd)
        
    def turn(self): #function to turn 90 degrees right
        self.wheel_cmd.vel_left = 0.4
        self.wheel_cmd.vel_right = -0.4
        self.publisher.publish(self.wheel_cmd) #turn half speed for accuracy
        time.sleep(0.15) #1s is first initialization for turn time
        self.publisher.publish(self.wheel_cmd)
        time.sleep(0.25)        
        self.wheel_cmd.vel_left = 0.0
        self.wheel_cmd.vel_right = 0.0
        self.publisher.publish(self.wheel_cmd)

    	
    
    
    def callback(self, msg): #set for half of the square currently

        #uses the fsm_state to perform our movement when the lane following demo is started
        if(msg.state == 'LANE_FOLLOWING'):

            time.sleep(2)
            self.forward()
            time.sleep(2)
            self.turn()
            time.sleep(2)
            self.forward()
            time.sleep(2)
            self.turn()
            
            
            
        if(msg.state == 'NORMAL_JOYSTICK_CONTROL'):
            self.wheel_cmd.vel_left = 0.0
            self.wheel_cmd.vel_right = 0.0
            self.publisher.publish(self.wheel_cmd)
            
            
if __name__ == '__main__':
    rospy.init_node('onemetersquare')
    agent = RosAgent()
    rospy.spin()
    

