# Experimental robotic assignment 1

## Introduction

<p align="center">
  <img src="https://github.com/Matt98x/Experimental_assignment1/blob/main/Images/Finite_state_machines.PNG?raw=true "Title"">
</p>
<p align="center">
  UML scheme. In order to inspect the details, please click on the image
</p>

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
	- combination of at maximum 2 commands with conjuctions of "and" or "or":
		- the "and" two equal commands, apart "play", makes them executing seguentially while the "or" execute just the first composed
		- between "go to" and "point to", if an and conjuction is provided, "go to" has priority and the other follows, if "or" only "go to" is executed
		- if a "play" is in second place with "and", only the "play" command is executed
		- if a "play" is conjuncted with "or" with any other command apart form itself, only "play" is executed
		- if a "play" is conjucted with "and" or "or" with itself, it will only be executed one
* The robot can receive any command while executing one in Play state but the ones given are neither executed nor stored.
* The robot can receive any command while in sleep state but the ones given are neither executed nor stored.
* Sleep preempt any other state when it starts.
* From Sleep you can only transition to Normal.
* The only command that can be received in Normal is "play".
* Two predifined positions inside the map are "Owner" and "Home", which cannot be changed during the execution, andcan be used instead of coordinates in giving commands.

