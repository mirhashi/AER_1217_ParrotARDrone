#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
import math 

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist


class PositionController(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here for position controller
    def __init__ (self):
		self.Kp_x = 1
		self.Kp_y = 1
		self.Kv_x = 1
		self.Kx_y = 1
		self.g = -9.81
		self.z_acc = 0

		self.x_i = 0
		self.t_i = 0 
		self.y_i = 0

		self.xdes_i = 0
		self.tdes_i = 0 
		self.ydes_i = 0

   
    def actual_vel_calc(self,X,Y,t):
		X_dot = (X - self.x_i)/(t - self.t_i)
		Y_dot = (Y - self.y_i)/(t - self.t_i)
		self.x_i = X;
		self.y_i = Y;
		self.t_i = t; 
		return X_dot,Y_dot

    def desired_vel_calc(self,X,Y,t):
		X_dot = (X - self.x_i)/(t - self.t_i)
		Y_dot = (Y - self.y_i)/(t - self.t_i)
		self.xdes_i = X;
		self.ydes_i = Y;
		self.tdes_i = t; 
		return X_dot,Y_dot


    def pos_cont(self,x,y,t, xdes,ydes,tdes, theta, phi):
		x_dot, y_dot = self.actual_vel_calc(x,y,t)
		xdes_dot, ydes_dot = self.desired_vel_calc(xdes,ydes,tdes)

		x_acc = self.Kv_x*(xdes_dot - x_dot) + self.Kp_x*(xdes - x)
		y_acc = self.Kv_y*(ydes_dot - y_dot) + self.Kp_y*(ydes - y)

		f = (self.z_acc + self.g)/(cos(theta)*cos(phi))

		phi_c = np.asin(-y_acc/f)
		theta_c = np.asin(-x_acc/(f*cos(phi_c))) 

		return phi_c, theta_c
