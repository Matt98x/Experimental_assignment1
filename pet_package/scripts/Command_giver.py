#!/usr/bin/env python

## @file Command_giver.py
# @brief Component that gives command either randomly generated or given by a user, switching between the two
#
# Details: This component is the one that inputs commands inside the pet_simulation, either it randomly generates random string command or receives them from the user
#

## Libraries declaration
import rospy
from std_msgs.msg import String
import math
import sys
import random

## Main function declaration
if __name__ == '__main__':
	## Init the ros node
	rospy.init_node("Command_generator")
	# Declaration of the subscriber
	pub=rospy.Publisher("commander",String,queue_size=10)
