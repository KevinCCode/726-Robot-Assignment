#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "math.h"
#define PI 3.1416

geometry_msgs::Twist velocityCommand; 

float frontReading = 0;
//Initial wall driving code
float wallInit = 0;
float largeComp = 0;
float largeAngle = 0;
//Positional Readings
float xP = 0;
float yP = 0;
float zP = 0;
float wP = 0;
float angle = 0;
/*
The scan subscriber call back function
To understand the sensor_msgs::LaserScan object look at
http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
*/

//Gets the readings of positon and also the angle orientation of the robot
//Is mainly used for obtaining the angle of the robot with reference to the world frame
void positionReading(const nav_msgs::Odometry::ConstPtr& msg){
	xP = msg->pose.pose.orientation.x;
	yP = msg->pose.pose.orientation.y; 
	zP = msg->pose.pose.orientation.z;
	wP = msg->pose.pose.orientation.w; 
	double sin = 2*(wP*zP + xP*yP);
	double cos = 1 - 2*(pow(yP,2) + pow(zP,2));
	angle = atan2(sin,cos); //the additional 1.5708 will make the left point 0 and the right point 3.1416 radians to match the robot car
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
// Compute the number of data points
// max angle and min angle of the laser scanner divide by the increment angle of each data point
float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
//*******************************
	//GET ANGLE	
	if(wallInit == 0){
		//Obtain the datapoint angle and position of the furtherest point away from the robot (which should technically mean that it has hit a wall)
		for(int i = 0; i < rangeDataNum; i++){
			if((laserScanData->ranges[i] > largeComp)){
				largeComp = laserScanData->ranges[i];
				largeComp = i;
			}	
		}
		largeAngle = (laserScanData->angle_increment*largeComp) - 1.57; //obtain the angle of furtherest position
		wallInit = 1; //set the flag to high so that the code does not enter this chunk again
	}
	//FACE THAT ANGLE
	if(wallInit == 1){
			//IMPORTANT NOTE: This code assumes that the robot car is always facing "upwards" when it is being used
			//Also by default the odometer reading is 0, with positive being on the left and negative being on the right
			if((largeAngle > 0) && (wallInit == 1)){ //The object is on the left side
				//use -ve
				ROS_INFO("Larger Loop");
				if(angle < (largeAngle + 0.1)){
				ROS_INFO("Enter Larger While");
					ROS_INFO("Angle = %.2f", angle);
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.1; //turn to the left side at 0.1 rad/s
				} else if( (angle < (largeAngle + 0.2)) && (angle > (largeAngle - 0.2)) ){
				ROS_INFO("Point Reached");
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.0;
					wallInit = 2;
				}
			} else if((largeAngle < 0) && (wallInit == 1)) { //The object is on the right side
				//use +ve 
				ROS_INFO("Smaller Loop");
				if(angle > (largeAngle - 0.1)){ //the robot will believe its angle is 0
				ROS_INFO("Entering Smaller While");
					ROS_INFO("Angle = %.2f", angle);
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = -0.1; //turn to the right side at 0.1 rad/s
				} else if( (angle < (largeAngle + 0.2)) && (angle > (largeAngle - 0.2)) ){
					ROS_INFO("Point Reached");
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.0;
					wallInit = 2;
				}
			} else {
				ROS_INFO("Ignored");
			}
		}		
//******************************	
}

int main (int argc, char **argv)
{	
	ros::init(argc, argv, "pioneer_laser_node");	// command line ROS arguments
	ros::NodeHandle my_handle;	// ROS comms access point

	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);

	/*
	subscribe to the scan topic and define a callback function to process the data
	the call back function is called each time a new data is received from the topic
	*/
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);
	ros::Subscriber odo_sub_object = my_handle.subscribe("/odom", 100, positionReading);

	ros::Rate loop_rate(10);// loop 10 Hz
	
	while(ros::ok()) // publish the velocity set in the call back
	{
		ROS_INFO("Angle = %.2f", angle);
		ROS_INFO("largeAngle = %.2f", largeAngle);
		ROS_INFO("wallInit Value = %.2f", wallInit);
		ros::spinOnce();
		loop_rate.sleep();

		// publish to the twist to the topic
		vel_pub_object.publish(velocityCommand);
	}

	return 0;
}
