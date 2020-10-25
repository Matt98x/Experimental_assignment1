#!/usr/bin/env python

## @file Pet_logic.py
# @brief Pet logic
#
# Details: This component is the one that receives command, convert the command in a more compliant format and
# change state
#

##! Libraries declaration
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from robot_pose_ekf.srv import GetStatus
import math
import sys
import random
import time

##! Variable declaration
command=[] #
msg_received=1 #if a message is received

##! @fn srvCallback
def srvCallback(req):
	# Service to send the command to Pet_behaviours
	global command
	temp=command
	command=""	
	return temp

##! @fn comCallback
# @brief The callback that receive the command and convert them to a compliant format
def comCallback(msg):
	global msg_received,command
	msg_received=rospy.get_param('in_course')
	state=rospy.get_param('state')
	# If it is not processing any other message
	# If a message was received parse it
	if not msg_received and state==2:
		temp_string="";
		# separate all commands with "and" delimeter
		tlist=str(msg.data).split(" and ")
		# for every command, parse it
		for i in range(len(tlist)):
			# split for every space
			temp=tlist[i].split(" ")
			for j in range(len(temp)-2):
				if temp[j+2]=="home" or temp[j+2]=="owner":
					rospy.loginfo(str(temp[j+2]))
					temp_string+=str(rospy.get_param(str(temp[j+2])+str("/")+str("x")))
					temp_string+=" "
					temp_string+=str(rospy.get_param(str(temp[j+2])+str("/")+str("y")))
				else:
					temp_string+=str(temp[j+2])
				if j<len(temp)-3:
					temp_string+=" "
			if i<len(tlist)-1:
					temp_string+="|"
			command=temp_string


##! @fn main
# Main function declaration
if __name__ == '__main__':

	# Init the ros node
	rospy.init_node("pet_logic")
	
	# Declaration of the subscriber
	rospy.Subscriber("commander",String,comCallback)
	# Declaration of the service
	s = rospy.Service('commandsrv', GetStatus, srvCallback)
	rate = rospy.Rate(5) # 10hz
	while not rospy.is_shutdown():
		
		rate.sleep()

