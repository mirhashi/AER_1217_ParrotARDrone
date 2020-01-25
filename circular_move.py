#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

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

       	self.center_x = 0         #trajectory centered at (x,y)=(0,0)
       	self.center_y = 0
       	self.angle = radians(0)   #starting angle is 0 degrees, 0 radians
       	self.radius = 1           #radius of circle = 1m
       	self.speed = 0.1          #angular velocity rad/s
       	self.altitude_z = 0.5     #altitude = 0.5m 
       	

    def circular_move(self):

        #starting position x,y,z
        x = self.center_x + self.radius * cos(self.angle)
        y = self.center_y - self.radius * sin(self.angle)
        z = self.altitude_z

        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():

            #new position x,y,z, and angle
            self.angle += self.speed  #angle becomes equal to pi radians in
                                      #pi/0.1 iterations (almost 32)
            x = x + self.radius * self.speed * math.cos(self.angle + pi/2.0)        #calculating new x
            y = y - self.radius * self.speed * math.sin(self.angle + pi/2.0) 
           
            if self.angle <= pi:  #as angle increases from 0 to pi,z increases
                z = z + (1/pi * self.speed)   #at angle pi radians, z should be 1.5m
            else:         #as angle increases from pi to 2pi,z decreases 
                z = z - (1/pi * self.speed)
                if self.angle >= 2 * pi:
                    self.angle = 0

            self.desired_position_msg.transform.translation.x = x
            self.desired_position_msg.transform.translation.y = y
            self.desired_position_msg.transform.translation.z = z

            self.desired_position_msg.header.stamp = rospy.Time.now()

            self.pub_des_position.publish(self.desired_position_msg)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('desired_position')
    pos_generator = ROSDesiredPositionGenerator()
    pos_generator.circular_move()
