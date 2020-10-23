/**
* \file simulator_node.cpp
* \brief Test component of the pet state machine output to the turtlesim environment
* 
* Details: This component simulate the command from the pet state machine and relays them to the turtlesim
* environment reading the current position of the turtlesim pet
*/

//! Libraries declaration
#include<ros/ros.h>
#include<geometry_msgs/Twist>
#include<turtlesim/Pose>
#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<math.h>

//! Global variable declaration
int x,y; //!<coordinates of the 
float theta;
/**
* \fn void turtlecallback(const turtlesim/Pose)
* \brief Callback function for the turtle position in the turtlesim environment
*
* \param data for the current turtle position
*/

void turtlecallback(const turtlesim::Pose::ConstPtr& msg)
{
	x=msg.linear.x;
	y=msg.linear.y;
}

//! Main function body
int main(int argc,char **argv){
	int dx,int dy;
	int sx,sy;
	//! Declaration phase
	//! To initiate the ros component
	ros::init(argc,argv,"state_out_sim");
	//! Declaration for the node handle
	ros::NodeHandle n;
	//! Service declaration (for turtlesim/teleport_relative)
	ros::ServiceClient client = n.serviceClient<turtlesim::TeleportRelative>("pet_simulator");
	//! Service declaration (for turtlesim/teleport_absolute)
	ros::ServiceClient client_init = n.serviceClient<turtlesim::TeleportAbsolute>("pet_initiator");
	//! Subscriber declaration
	ros::Subscriber sub = n.subscribe("turtlesim", 1000, chatterCallback);
	//! Loop rate (rate is one of the test parameter)
	ros::Rate loop_rate(10);
	

	turtlesim::TeleportAbsolute srv;
	srv.request.x=0;
	srv.request.y=0;
	srv.request.theta=0;
	if (client_init.call(srv))
	{
	  ROS_INFO("Successful request");
	}
	else
	{
	  ROS_ERROR("Failed to call service");
	  return 1;
	}

	//! Main loop
	  while (ros::ok())
	  {
	    //! Message construction
	    turtlesim::TeleportRelative srv;
	    dx=floor(rand%3)-1;
	    dy=floor(rand%3)-1;
	    while (!(x+dx<11 && x+dx>0)){
		dx=floor(rand%3)-1;
	    }
	    while (!(y+dy<11 && y+dy>0)){
		dy=floor(rand%3)-1;
	    }
	    srv.request.linear+sqrt(pow(dx,2)+pow(dy,2));
	    srv.request.angular+atan2(float(dy),float(dx));
	    //! Service call and error handling
	    if (client.call(srv))
	    {
	      ROS_INFO("Successful request");
	    }
	    else
	    {
	      ROS_ERROR("Failed to call service");
	      return 1;
	    }
	    ros::spinOnce();

	    loop_rate.sleep();
	  }
}
