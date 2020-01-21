#!/usr/bin/env python2

"""
ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from position_controller import PositionController


class ROSControllerNode(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here to define node publishers and subscribers
    # publish to /cmd_vel topic
    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
    def __init__ (self):

    	# Publishers
        self.pub_vel_cmd = rospy.Publisher('/cmd_vel_RHC', 
                                            Twist, queue_size=32)

        #Subscriber
        self.sub_vicon = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre', 
                                                TransformStamped,
                                                self.update_quadrotor_state)
        self.sub_desired_pos = rospy.Subscriber('/desired_position', 
                                                TransformStamped,
                                                self.update_desired_pos)
    
        self.actual_pos = TransformStamped()

        self.desired_pos = TransformStamped()

        self.vel_cmd_msg = Twist()

        self.pub_prop_vel = rospy.Timer(rospy.Duration(1/10.0), self.send_vel_cmd)

        self.pos_controller = PositionController()

    def update_quadrotor_state(self, transfrom_stamped_msg):
    	self.actual_pos = transfrom_stamped_msg

    def send_vel_cmd(self, event):
        xdes = self.desired_pos.transform.translation.x
        ydes = self.desired_pos.transform.translation.y
        tdes = self.desired_pos.header.stamp

        xact = self.actual_pos.transform.translation.x
        yact = self.actual_pos.transform.translation.y
        tact = self.actual_pos.header.stamp 
        
        rotation = (self.actual_pos.transform.rotation.x,
                    self.actual_pos.transform.rotation.y,
                    self.actual_pos.transform.rotation.z,
                    self.actual_pos.transform.rotation.w)
        
        euler_angle = euler_from_quaternion(rotation)

        phi_act = euler_angle[0]
        theta_act = euler_angle[1]

        phi_c, theta_c = self.pos_controller.pos_cont(xact, yact, tact, xdes,ydes,tdes, theta_act, phi_act)

        vel_cmd_msg.linear.x  = phi_c
        vel_cmd_msg.linear.y  = theta_c

    	self.pub_vel_cmd.publish(self.vel_cmd_msg)

    def update_desired_pos(self, transformstamped_msg):
    	self.desired_pos = transformstamped_msg


if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('ros_interface')
    ROSControllerNode()
    rospy.spin()


