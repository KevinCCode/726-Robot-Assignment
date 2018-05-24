#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "math.h"
#define PI 3.1416

geometry_msgs::Twist velocityCommand; 
//State Variable
float state = 0;
//Initial Angle Readings
float largeComp = 0;
float largeAngle = 0;
//Position variables
float quadrantVal = 0;
float frontReading = 0;
//Spin code
float angleSum = 0; //this sums up all the angles
//float originalAngle = 0; //hold onto this value as the original value, this is used to ensure that it is static all the time
//float moveStartAngle = 0; //this starts the movement process
//float sCase = 0; //sCase covers the case of whether the angle is around 3.05 and higher since +0.1 will make it the negative equivalent
//float angleLatch = 0; //latch onto a single angle
//QuadrantVal Guide, each value corresponds to the quadrant of the map that the robot believes that it is in
//Though this is generally moreso used to define which way the robot car is facing and where it should turn to afterwards
//  _ _ _ _ _ _
// |2    |1    |
// |_ _ _|_ _ _|
// |3    |4    |
// |_ _ _|_ _ _|
//Odometer Reading Variables
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
frontReading = laserScanData->ranges[rangeDataNum/2]; //take the reading directly in front of the robot	



//**********MOVE TO FIRST CORNER*******************



	//GET ANGLE OF FIRST CLEAR/OPEN POINT
	if(state == 0){
		//Obtain the datapoint angle and position of the furtherest point away from the robot (which should technically mean that it has hit a wall)
		for(int i = 0; i < rangeDataNum; i++){
			if((laserScanData->ranges[i] > largeComp)){
				largeComp = laserScanData->ranges[i];
				largeComp = i;
			}	
		}
		largeAngle = (laserScanData->angle_increment*largeComp) - 1.57; //obtain the angle of furtherest position

		//Setting the quadrant values to tell the robot which part of the map it is in		
		if((largeAngle < 0.1) && (largeAngle > -1.56)){
			quadrantVal = 1; //around 0 to -1.57	
		} else if((largeAngle > 0.1) && (largeAngle < 1.56)){
			quadrantVal = 2; //around 0 to 1.57
		} else if((largeAngle > 1.57) && (largeAngle < 3.13)){
			quadrantVal = 3; //around 1.57 to 3.14
		} else {
			quadrantVal = 4; //around -1.57 to -3.14
		}
		state = 1; //set the flag to high so that the code does not enter this chunk again
	}

	//FACE THE DIRECTION OF THAT CLEAR/OPEN POINT
	if(state == 1){
			//IMPORTANT NOTE: This code assumes that the robot car is always facing "upwards" when it is being used
			//Also by default the odometer reading is 0, with positive being on the left and negative being on the right
			if((largeAngle > 0) && (state == 1)){ //The object is on the left side
				//use -ve
				//ROS_INFO("Larger Loop");
				if(angle < (largeAngle + 0.1)){
				//ROS_INFO("Enter Larger While");
					//ROS_INFO("Angle = %.2f", angle);
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.1; //turn to the left side at 0.1 rad/s
				} else if( (angle < (largeAngle + 0.2)) && (angle > (largeAngle - 0.2)) ){
				//ROS_INFO("Point Reached");
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.0;
					largeComp = 0;
					largeAngle = 0;
					state = 2;
				}
			} else if((largeAngle < 0) && (state == 1)) { //The object is on the right side
				//use +ve 
				///ROS_INFO("Smaller Loop");
				if(angle > (largeAngle - 0.1)){ //the robot will believe its angle is 0
				//ROS_INFO("Entering Smaller While");
					//ROS_INFO("Angle = %.2f", angle);
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = -0.1; //turn to the right side at 0.1 rad/s
				} else if( (angle < (largeAngle + 0.2)) && (angle > (largeAngle - 0.2)) ){
					//ROS_INFO("Point Reached");
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.0;
					largeComp = 0;
					largeAngle = 0;
					state = 2; //move the trigger to a new value 
				}
			} else {
				ROS_INFO("Angle Error!");
			}
		}	

	//MOVE TO THAT POINT UNTIL YOU REACH THE WALL
	if(state == 2){
		//ROS_INFO("FrontReading = %.2f", frontReading);
		if(frontReading > 0.4){
			//ROS_INFO("Entering Move");
			velocityCommand.linear.x = 0.5; //stop forward movement 
			velocityCommand.angular.z = 0; //turn left
		} else {
			//Come to a complete stop
			velocityCommand.linear.x = 0;
			velocityCommand.angular.z = 0; 
			state = 3; //set the trigger to a new value
		}
	}

	//STRAIGHTEN AND FACE A CORNER (1st Corner)
	if(state == 3){
		//ROS_INFO("Quadrant 1");
		if(quadrantVal == 1){ //if we are in quadrant 1, we will try to align back to 0 from a negative value
			if(angle < 0){ //we cannot exactly get to 0, so we aim for 0.1 instead to ensure we are going away from the wall, as with before
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = 0.1;
			} else {
				//Come to a complete stop
				velocityCommand.linear.x = 0;
				velocityCommand.angular.z = 0; 
				state = 4; //leave state
			}
		} else if(quadrantVal == 2){
		//ROS_INFO("Quadrant 2"); //moving back towards 0 from positive
			if(angle > 0){ //we cannot exactly get to 0, so we aim for 0.1 instead to ensure we are going away from the wall
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = -0.1;
			} else {
				//Come to a complete stop
				velocityCommand.linear.x = 0;
				velocityCommand.angular.z = 0; 
				state = 4;
			}
		} else if(quadrantVal == 3){
		//ROS_INFO("Quadrant 3"); //moving to positive from negative
			if(angle < 3.13){ //we cannot exactly get to -3.14, so we have to aim for POSITIVE 3.13 instead to ensure we are going away from the wall
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = 0.1;
			} else {
				//Come to a complete stop
				velocityCommand.linear.x = 0;
				velocityCommand.angular.z = 0; 
				state = 4;
			}
		} else if(quadrantVal == 4){
		//ROS_INFO("Quadrant 4"); // moving to negative from positive
			if(angle > -3.13){ //we cannot exactly get to 0, so we aim for 0.1 instead to ensure we are going away from the wall
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = -0.1;
			} else {
				//Come to a complete stop
				velocityCommand.linear.x = 0;
				velocityCommand.angular.z = 0; 
				state = 4;
			}
		} else {
			ROS_INFO("Quadrant Error!");
		}
	}
	
	//DRIVE TO THAT CORNER (1st Corner)
	if(state == 4){ 
		if(frontReading > 0.4){
			velocityCommand.linear.x = 0.5;
			velocityCommand.angular.z = 0;
		} else {
			velocityCommand.linear.x = 0;
			velocityCommand.angular.z = 0;	
			state = 5;		
		}
	}

	//FIRST CORNER SPIN INSTANCE
	if(state == 5){
			if((angleSum >= 0) && (angleSum < 124)){
				//Spin 
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = 0.5;
				angleSum = angleSum + 1;
				ROS_INFO("Angle Sum: %.2f", angleSum);
			} else {
				angleSum = 0;
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = 0.0;
				state = 6;
			}
		}

//***************************************

//********2ND CORNER*********************



	//ADJUST FOR 2ND CORNER
	if(state == 6){
		//Obtain the datapoint angle and position of the NEWEST point away from the robot (which technically shows the "open way" for the robot to go to
		for(int i = 0; i < rangeDataNum; i++){
			if((laserScanData->ranges[i] > largeComp)){
				largeComp = laserScanData->ranges[i];
				largeComp = i;
			}	
		}
		largeAngle = (laserScanData->angle_increment*largeComp) - 1.57; //obtain the angle of furtherest position		
		state = 7;
	}

	//FACE THE DIRECTION OF THAT NEW CLEAR/OPEN POINT (FOR 2ND CORNER)
	if(state == 7){
			//IMPORTANT NOTE: This code assumes that the robot car is always facing "upwards" when it is being used
			//Also by default the odometer reading is 0, with positive being on the left and negative being on the right
			if((largeAngle > 0) && (state == 7)){ //The object is on the left side
				//use -ve
				//ROS_INFO("Larger Loop");
				if(angle < (largeAngle + 0.1)){
				//ROS_INFO("Enter Larger While");
					//ROS_INFO("Angle = %.2f", angle);
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.1; //turn to the left side at 0.1 rad/s
				} else if( (angle < (largeAngle + 0.2)) && (angle > (largeAngle - 0.2)) ){
				//ROS_INFO("Point Reached");
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.0;
					largeComp = 0;
					largeAngle = 0;
					state = 8;
				}
			} else if((largeAngle < 0) && (state == 7)) { //The object is on the right side
				//use +ve 
				///ROS_INFO("Smaller Loop");
				if(angle > (largeAngle - 0.1)){ //the robot will believe its angle is 0
				//ROS_INFO("Entering Smaller While");
					//ROS_INFO("Angle = %.2f", angle);
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = -0.1; //turn to the right side at 0.1 rad/s
				} else if( (angle < (largeAngle + 0.2)) && (angle > (largeAngle - 0.2)) ){
					//ROS_INFO("Point Reached");
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.0;
					largeComp = 0;
					largeAngle = 0;
					state = 8; //move the trigger to a new value 
				}
			} else {
				ROS_INFO("Angle Error!");
			}
		}	

	//DRIVE TO THAT CORNER (2nd Corner)
	if(state == 8){ 
		if(frontReading > 0.4){
			velocityCommand.linear.x = 0.5;
			velocityCommand.angular.z = 0;
		} else {
			velocityCommand.linear.x = 0;
			velocityCommand.angular.z = 0;	
			state = 9;		
		}
	}

	//SECOND CORNER SPIN INSTANCE
	if(state == 9){
			if((angleSum >= 0) && (angleSum < 124)){
				//Spin 
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = 0.5;
				angleSum = angleSum + 1;
				ROS_INFO("Angle Sum: %.2f", angleSum);
			} else {
				angleSum = 0;
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = 0.0;
				state = 10;
			}
		}


//************************************


//**********3RD CORNER****************


	//ADJUST FOR 3RD CORNER
	if(state == 10){
		//Obtain the datapoint angle and position of the NEWEST point away from the robot (which technically shows the "open way" for the robot to go to
		for(int i = 0; i < rangeDataNum; i++){
			if((laserScanData->ranges[i] > largeComp)){
				largeComp = laserScanData->ranges[i];
				largeComp = i;
			}	
		}
		largeAngle = (laserScanData->angle_increment*largeComp) - 1.57; //obtain the angle of furtherest position		
		state = 11;
	}

	//FACE THE DIRECTION OF THAT NEW CLEAR/OPEN POINT (FOR 3RD CORNER)
	if(state == 11){
			//IMPORTANT NOTE: This code assumes that the robot car is always facing "upwards" when it is being used
			//Also by default the odometer reading is 0, with positive being on the left and negative being on the right
			if((largeAngle > 0) && (state == 11)){ //The object is on the left side
				//use -ve
				//ROS_INFO("Larger Loop");
				if(angle < (largeAngle + 0.1)){
				//ROS_INFO("Enter Larger While");
					//ROS_INFO("Angle = %.2f", angle);
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.1; //turn to the left side at 0.1 rad/s
				} else if( (angle < (largeAngle + 0.2)) && (angle > (largeAngle - 0.2)) ){
				//ROS_INFO("Point Reached");
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.0;
					largeComp = 0;
					largeAngle = 0;
					state = 12;
				}
			} else if((largeAngle < 0) && (state == 11)) { //The object is on the right side
				//use +ve 
				///ROS_INFO("Smaller Loop");
				if(angle > (largeAngle - 0.1)){ //the robot will believe its angle is 0
				//ROS_INFO("Entering Smaller While");
					//ROS_INFO("Angle = %.2f", angle);
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = -0.1; //turn to the right side at 0.1 rad/s
				} else if( (angle < (largeAngle + 0.2)) && (angle > (largeAngle - 0.2)) ){
					//ROS_INFO("Point Reached");
					velocityCommand.linear.x = 0.0;
					velocityCommand.angular.z = 0.0;
					largeComp = 0;
					largeAngle = 0;
					state = 12; //move the trigger to a new value 
				}
			} else {
				ROS_INFO("Angle Error!");
			}
		}
	
	//DRIVE TO THAT CORNER (3rd Corner)
	if(state == 8){ 
		if(frontReading > 0.4){
			velocityCommand.linear.x = 0.5;
			velocityCommand.angular.z = 0;
		} else {
			velocityCommand.linear.x = 0;
			velocityCommand.angular.z = 0;	
			state = 9;		
		}
	}

	//THIRD CORNER SPIN INSTANCE
	if(state == 9){
			if((angleSum >= 0) && (angleSum < 124)){
				//Spin 
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = 0.5;
				angleSum = angleSum + 1;
				ROS_INFO("Angle Sum: %.2f", angleSum);
			} else {
				angleSum = 0;
				velocityCommand.linear.x = 0.0;
				velocityCommand.angular.z = 0.0;
				state = 10;
			}
		}

//***************************

//Ending bracket
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
		//ROS_INFO("Quadrant = %.2f", quadrantVal);
		ROS_INFO("State = %.2f", state);
		//ROS_INFO("angleLatch = %.2f", angleLatch);
		ros::spinOnce();
		loop_rate.sleep();

		// publish to the twist to the topic
		vel_pub_object.publish(velocityCommand);
	}

	return 0;
}
