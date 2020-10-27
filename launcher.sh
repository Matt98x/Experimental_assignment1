#!/bin/bash

gnome-terminal -x sh -c "roscore; bash"
roslaunch pet_package launcher.launch

