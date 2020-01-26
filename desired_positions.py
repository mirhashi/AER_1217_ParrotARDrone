#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
from numpy import pi

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator

from geometry_msgs.msg import TransformStamped

class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""
    # write code here for desired position trajectory generator
    def __init__ (self):

    	# Publishers
        self.pub_des_pos = rospy.Publisher('/desired_position', 
                                            TransformStamped, queue_size=32)

        self.desired_position_msg = TransformStamped()

       	self.point1 = np.array([-1, -1, 1])
       	self.point2 = np.array([1, 1, 2])

       	self.position = np.array([-1, -1, 1])

       	self.center_x = 0         #trajectory centered at (x,y)=(0,0)
       	self.center_y = 0
       	self.angle = np.radians(0)   #starting angle is 0 degrees, 0 radians
       	self.radius = 1           #radius of circle = 1m
       	self.speed = 0.1          #angular velocity rad/s
       	self.z_min = 0.5     #altitude = 0.5m 
       	self.z_max = 1.5

    def linear_move(self):

    	delta =  (self.point2-self.point1)/100.0

    	rate = rospy.Rate(50)

    	direction = 1

       	while not rospy.is_shutdown():

       		
       		self.position = self.position + direction * delta

       		if (self.position > self.point2).all() or (self.position < self.point1).all():
       			direction = -1*direction
  
       		self.desired_position_msg.transform.translation.x = self.position [0]
       		self.desired_position_msg.transform.translation.y = self.position [1]
       		self.desired_position_msg.transform.translation.z = self.position [2]
       		self.desired_position_msg.header.stamp = rospy.Time.now()

       		self.pub_des_pos.publish(self.desired_position_msg)


       		rate.sleep()


    def circular_move(self):

        #starting position x,y,z
        x = self.center_x + self.radius * np.cos(self.angle)
        y = self.center_y + self.radius * np.sin(self.angle)

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():

            #new position x,y,z, and angle
            self.angle += self.speed  
            
            x = self.center_x + self.radius * np.cos(self.angle)  
            y = self.center_y + self.radius * np.sin(self.angle) 
           
            if self.angle <= pi:  
                z =  self.z_min+(self.z_max - self.z_min)*(1 / pi * self.angle)   
            else:         
                z =  self.z_max - (self.z_max - self.z_min)*(1 / pi * (self.angle - pi))
                if self.angle >= 2 * pi:
                    self.angle = 0

            self.desired_position_msg.transform.translation.x = x
            self.desired_position_msg.transform.translation.y = y
            self.desired_position_msg.transform.translation.z = z

            self.desired_position_msg.header.stamp = rospy.Time.now()

            self.pub_des_pos.publish(self.desired_position_msg)

            rate.sleep()
       	

    

if __name__ == '__main__':
    rospy.init_node('desired_position')
    pos_generator = ROSDesiredPositionGenerator()
    pos_generator.linear_move()
