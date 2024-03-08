#!/usr/bin/env python

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

import rospy




class maze_solver():  
    def __init__(self):

        rospy.init_node('solver') #Intializing the solver node

#initialisations:
        

        #Frequency of operation.
        self.sample_rate = 10.0 #10 Hz (better to keep it around 50)
        self.scan = [0, 0, 0, 0, 0] #for storing the laser scan data    
        
        self.LeftWheel = Float64()
        self.LeftWheel.data = 0.0

        self.RightWheel = Float64()
        self.RightWheel.data = 0.0
        

    #Publishers
        self.LeftWheelPub = rospy.Publisher('/mobot/Left_wheel_velocity/command', Float64, queue_size=1)
        self.RightWheelPub = rospy.Publisher('/mobot/Right_wheel_velocity/command', Float64, queue_size=1)
        
        
    #Subscribers
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        


#callbacks

  
    def laser_callback(self, msg):
        self.scan = msg.ranges

#algorithm

    def solving_algo(self):
    	
    	if self.scan[2] < 1:
    		if self.scan[0] > self.scan[4]:
    			self.RightWheel.data = self.LeftWheel.data - 3.0

	    	elif self.scan[0] < self.scan[4]:
	    		self.LeftWheel.data = self.RightWheel.data - 3.0

    	elif self.scan[1] < 1:
    		self.LeftWheel.data = self.RightWheel.data - 3.0

    	elif self.scan[3] < 1:
    		self.RightWheel.data = self.LeftWheel.data - 3.0


    	else:

    		self.LeftWheel.data = 3.0
    		self.RightWheel.data = 3.0


    	
#sending the data to publish

    	self.LeftWheelPub.publish(self.LeftWheel)
    	self.RightWheelPub.publish(self.RightWheel)
        






if __name__ == '__main__':

    solver = maze_solver() #Creating solver object
    r = rospy.Rate(solver.sample_rate) #100Hz frequency
    while not rospy.is_shutdown():

        try:
            solver.solving_algo()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
