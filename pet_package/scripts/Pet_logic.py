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
msg_received=0 #if a message is received

##! @fn srvCallback
def srvCallback(req):
	global command
	string=""
	for i in range(len(command)):
		string.concatenate(str(command(i)))
		if i<len(command)-1:
			string.concatenate(" ")
	return str(command)

##! @fn comCallback
# @brief The callback that receive the command and convert them to a compliant format
def comCallback(data):
	global command,msg_received
	if msg_received==0:
		temp1=data.split("and") # if there is an "and"
		if len(temp1)>len(data):
			temp=temp1
		temp2=data.split("or") # if there is an "or"
		if len(temp2)>len(data):
			temp=temp2
		com=[];
		for i in range(len(temp)):	
			elem=temp.split(" ")
			if elem(1)=="play":
				rospy.set_param("state",2)
			elif elem(1)=="go":
				goto=[int(elem(3)),int(elem(4))]
				com.concatenate(goto)
			elif elem(1)=="point":
				poto=[int(elem(3)),int(elem(4))]
				com.concatenate(poto)
		command=com
		msg_received=1
	state=rospy.get_param("state")
	if state==2:
		rospy.set_param('in_course',1)

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

