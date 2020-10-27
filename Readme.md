# Experimental robotic assignment 1

## Introduction

This assignment target is to build an ROS architecture to implement a robot, simulating a pet, that
interact with a human and moves in a discrete 2D environment.  
The pet has three states representing behaviours, that are simulated as a finite state machine with 3 states:  
* Play 
* Sleep 
* Normal  
  
These states determine the way the robot act inside the grid, whether moving randomly as in normal, going to targets determined by the user or simply sleeping in the home position.  
The robot will change between states randomly, eccept for play, which is received by the user.  

## Software Architecture

The software architecture consists in a pipeline, starting from the command generation and arriving to the pet simulation in turtlesim.  
Here we show the architecture image:  
<p align="center">
  <img src="https://github.com/Matt98x/Experimental_assignment1/blob/main/Images/Finite_state_machines.PNG?raw=true "Title"">
</p>
<p align="center">
  UML scheme. In order to inspect the details, please click on the image
</p>
The main components are:
* Random command generator(Command_giver.py): send a string representing the concatenation of one or more commands of the form: 'play'(to start the play state),'point to x y'(to simulate the pointing gesture to x y),'go to x y'(to simulate the voice command to x y)
* Pet Interpreter(Pet_logic.py): to interpret the string commands and translate them to a command list
* Pet behaviours(Pet_behaviours.py): that simulate behaviours as a finite state, in the already mentioned states 


## Installation and running procedure

* Download the package from the github repository
* Set the package in the src folder of the catkin workspace
* With the shell, get into the folder and run 
 '''.sh
	chmod +x launcher.sh
 '''
* Write ./launcher.sh


## Working assumptions

* The robot, simulating a pet, interact with a human and moves in a discrete 2D environment.
* Both the robot targets and its positions belongs exclusively to the map(11 by 11 grid)representing the 2D environment.
* The robot has 3 main states:
	- Play
	- Normal
	- Sleep
* The robot receive forms in strings with possible form:
	- "play"	
	- "go to x1 y1" (equivalent to voice command)
	- "point to x1 y1" (equivalent to pointing commands)
	- combination of commands with conjuctions of "and":
		- All the command after the play are executed
		- if a "play" is not in first place, only commands after the "play" command are executed
* The robot can receive any command while executing one in Play state but the ones given are neither executed nor stored.
* The robot can receive any command while in sleep state but the ones given are neither executed nor stored.
* Sleep preempt any other state when it starts.
* From Sleep you can only transition to Normal.
* The only command that can be received in Normal is "play".
* Two predifined positions inside the map are "Owner" and "Home", which cannot be changed during the execution, and can be used instead of coordinates in giving commands.

## System features and limitations

## Author and contacts
Matteo Palmas: matteo.palmas7gmail.com
