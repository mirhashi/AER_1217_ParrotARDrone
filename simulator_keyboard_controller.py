#!/usr/bin/env python

# The Keyboard Controller Node for controlling the drone in simulator

# This controller has keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib
import rospy
import numpy as np
from geometry_msgs.msg import Twist  	 # for sending commands to the drone

import signal

import sys, select, termios, tty

from time import sleep

from threading import Thread

def cmd_reset(pubCommand):
	"called when read times out"
	command = Twist()	

	pitch = 0
	roll = 0
	yaw_velocity = 0
	z_velocity = 0

	while not rospy.is_shutdown():
		sleep(0.1)
		command.linear.x  = pitch
		command.linear.y  = roll
		command.linear.z  = z_velocity
		command.angular.z = yaw_velocity
		pubCommand.publish(command)

def cmd_send(pubCommand):
	

	#initiate command
	command = Twist()
	rate = rospy.Rate(10)
	
	pitch = 0
	roll = 0
	yaw_velocity = 0
	z_velocity = 0
	
	while not rospy.is_shutdown():
		#signal.signal(signal.SIGALRM, interrupted)
		#command = signal.alarm(1)
		#print(command)

		key = getKey()

		if key == 'q':
			z_velocity = 0.5
			pitch = 0
			roll = 0
			yaw_velocity = 0
		elif key == 'a':
			z_velocity = -0.5
			pitch = 0
			roll = 0
			yaw_velocity = 0
		elif key == 'e':
			pitch = 0.5
			roll = 0
			yaw_velocity = 0
			z_velocity = 0
			
		elif key == 'd':
			pitch = -0.5
			roll = 0
			yaw_velocity = 0
			z_velocity = 0

		elif key == 'f':
			roll = -0.5
			yaw_velocity = 0
			z_velocity = 0
			pitch = 0

		elif key == 's':
			roll = 0.5
			yaw_velocity = 0
			z_velocity = 0
			pitch = 0

		elif key == 'r':
			yaw_velocity = -0.5
			roll = 0
			z_velocity = 0
			pitch = 0

		elif key == 'w':
			yaw_velocity = 0.5
			roll = 0
			z_velocity = 0
			pitch = 0

		command.linear.x  = pitch
		command.linear.y  = roll
		command.linear.z  = z_velocity
		command.angular.z = yaw_velocity
		
			
		#signal.alarm(0)
		pubCommand.publish(command)
		
		rate.sleep()

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def keyboard_capture():
	#initiate node
	rospy.init_node('keyboard_controller')
	
	#publish commands       	
	pubCommand = rospy.Publisher('cmd_vel_RHC', Twist, queue_size=10)	
	
	

	t1 = Thread(target=cmd_send, args=(pubCommand,))
	t2 = Thread(target=cmd_reset, args=(pubCommand, ))
	
	t1.start()
	t2.start()
	
	


if __name__=='__main__': 

	try:
		keyboard_capture()

	except rospy.ROSInterruptException:
		pass

	
