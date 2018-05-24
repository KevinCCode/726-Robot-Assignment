#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

geometry_msgs::Twist velocityCommand; 
//Conversion factor to convert radians to degrees
float offset = 0.3;
float convert = 180/M_PI; 
//Smallest range value
float smallest = 100; //big number to offset any potential
float smallPos = 0; //position in the rangeDatNum data points which details where the closest point is
float smallAngle = 0;
float smallPointX = 0;
float smallPointY = 0;
//Difference values
float initGradient = 1;
float rangeDiff = 0;
float rangeDiff2 = 0;
//First point of contact
float firstPointLength = 0;
float firstPointAngle = 0;
float iFirstPoint = 0;
float lastPointLength = 0;
float lastPointAngle = 0;
float iLastPoint = 0;
//Left point value
float leftPointLength = 0;
float leftPointAngle = 0;
float iLeftPoint = 0;
//Co-ordinates of points
float firstPointX = 0;
float firstPointY = 0;
float leftPointX = 0;
float leftPointY = 0;
//Gradient measuring variables
float indexPosition = 0; 
float pointIncr = 0; //the value by which to increment for measuring gradient
float cX = 0;
float cY = 0;
float cLength = 0;
float cAngle = 0;
//Comparison Gradients
float compGradient = 0;
float compGradient2 = 0;
float match = 0; //counter variable for checking matches
//Identified Shapes
float shapeIdentifier = 0; //0 is circle, 1 is square
//Measurement points
float radius = 0;
float length = 0;
//Calculated values for lengths before they are processed as either length or width through judging which is larger
float chordL = 0;
float chordH = 0;
float width = 0;
float finalX = 0;
float finalY = 0;
/*
The scan subscriber call back function
To understand the sensor_msgs::LaserScan object look at
http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
*/
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	// Compute the number of data points
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
	
	//Obtain the datapoint position of the closest point to the robot, which should always be the bottom most corner, when stationary
	for(int i = 0; i < rangeDataNum; i++){
		if((laserScanData->ranges[i] < smallest)){
			smallest = laserScanData->ranges[i];
			smallPos = i;
		}	
	}
	smallAngle = laserScanData->angle_increment*smallPos; //obtain the angle of this smallest position
	//Calculate X and Y co-ordinates of the closest point
	smallPointX = smallest*(cos(smallAngle));
	smallPointY = smallest*(sin(smallAngle));	
	
	//sweep from right to left and measure the range readings, if the current reading is SMALLER than the 
	//previous reading, then it indicates that it has come across the object's rightmost point
	for(int i = 1; i < rangeDataNum; i++){
		rangeDiff = laserScanData->ranges[i] - laserScanData->ranges[i-1]; //Find the difference between the current range versus the previous range
		if(rangeDiff < -0.1 && (i != 1)){ //-ve range difference indicates an object has interrupted usual range pattern
			firstPointLength = laserScanData->ranges[i];
			firstPointAngle = (laserScanData->angle_increment*i);
			iFirstPoint = i;
			break; //break out immediately when this occurs
		}
	}
	//Calculate X and Y co-ordinates of the first point
	firstPointX = firstPointLength*(cos(firstPointAngle));
	firstPointY = firstPointLength*(sin(firstPointAngle));

	//sweep from left to right and measure the range readings, if the current reading is SMALLER than the 
	//previous reading, then it indicates that it has come across the object's left point
	for(int i = rangeDataNum - 1; i > 0; i--){
		rangeDiff2 = laserScanData->ranges[i] - laserScanData->ranges[i+1]; //Find the difference between the current range versus the previous range
		if(rangeDiff2 < -0.1 && (i != 511)){ //-ve range difference indicates an object has interrupted usual range pattern
			leftPointLength = laserScanData->ranges[i];
			leftPointAngle = (laserScanData->angle_increment*i);
			iLeftPoint = i;
			break; //break out immediately when this occurs
		}
	}
	//Calculate X and Y co-ordinates of the first point
	leftPointX = leftPointLength*(cos(leftPointAngle));
	leftPointY = leftPointLength*(sin(leftPointAngle)); //storing the X is for posterity, Y is mainly used

	//Obtains the value by which to increment for gradient check, this is used for X and Y
	pointIncr = fabs(round(0.5*(smallPos - iFirstPoint))); //round brings it to the closest whole number
	
	initGradient = (firstPointY - smallPointY)/(firstPointX - smallPointX); //calculate expected gradient for square

	match = 0;
	//First comparison of gradient
	indexPosition = smallPos - (pointIncr); //index position of next length, to be used in conversion
	cLength = laserScanData->ranges[indexPosition]; //obtain the length at that indexPosition
	cAngle = laserScanData->angle_increment*indexPosition; //obtain the angle at that indexPosition
	//Obtain the x and y positions for comparing
	cX = cLength*(cos(cAngle));
	cY = cLength*(sin(cAngle));
	compGradient = (cY - smallPointY)/(cX - smallPointX);
	if((compGradient > (initGradient*0.95)) && (compGradient < (initGradient*1.05)))
	{
		match++;
	}
	//Second comparison of gradient
	compGradient2 = (firstPointY - cY)/(firstPointX - cX);
	if((compGradient2 > (initGradient*0.95)) && (compGradient < (initGradient*1.05)))
	{
		match++;
	}

	//If 2 instances of satisfactory results are recorded, then the shape is a square only if the Y values of the other 2 co-ordinates are the same, else it remains a circle
	if(match == 2){
		shapeIdentifier = 1; //crate
		width = 100*(sqrt(pow(firstPointX - smallPointX,2) + pow(firstPointY - smallPointY,2)));   
		length = 100*(sqrt(pow(leftPointX - smallPointX,2) + pow(leftPointY - smallPointY,2)));
		//Calculates the finalX and finalY by connecting two Y and two X values and finding the middle point
		finalX = ((firstPointX + leftPointX)/2)*0.1;
		finalY = ((firstPointY + leftPointY)/2) + offset;
	} else {
		shapeIdentifier = 0; //barrel
		chordL = firstPointX - leftPointX;
		chordH = firstPointY - smallPointY;
		radius = 100*((4*(chordH*chordH)) + (chordL*chordL))/(8*chordH);	
		//Radius added to the closest point, since the radius always equal
		finalY = (smallPointY + (0.01*radius)) + offset;
	}
	
	//Used at the very end to convert anything with an nan to a shapeIdentifier 2, which indicates no objec	
	if(isnan(finalY) == 1){
		shapeIdentifier = 2;
	}	

	//Reset the smallest variable to ensure no latching of values, i.e. ensuring that the value of smallest will update at each iteration
	smallest = 100;
}

int main (int argc, char **argv)
{	
	ros::init(argc, argv, "laserInfo");	// command line ROS arguments
	ros::NodeHandle my_handle;	// ROS comms access point

	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);

	/*
	subscribe to the scan topic and define a callback function to process the data
	the call back function is called each time a new data is received from the topic
	*/
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);

	ros::Rate loop_rate(10);// loop 10 Hz
	
	while(ros::ok()) // publish the velocity set in the call back
	{
		//ROS_INFO("Right Position -> x: [%.2f], y: [%.2f]", firstPointX, firstPointY);
		//ROS_INFO("Left Position -> x: [%.2f], y: [%.2f]", leftPointX, leftPointY);
		//ROS_INFO("Closest Position -> x: [%.2f], y: [%.2f]", smallPointX, smallPointY);	
		//ROS_INFO("Object Found!! Object ID - 1");		
		if(shapeIdentifier == 0){ //if the shape has been found to be a circle
			ROS_INFO("Object Found!! - ID: 1");
			ROS_INFO("Type: Barrel, Radius = %.2f cm, X: %.2f, Y: %.2f", radius, finalY, finalX); //local X and system Y and vice-versa
		} else if(shapeIdentifier == 1){
			ROS_INFO("Object Found!! - ID: 1");
			ROS_INFO("Type: Container, Length = %.2f cm, Width = %.2f cm, X: %.2f, Y: %.2f", length, width, finalY, finalX); //Note: The local X is system Y and local Y and system X 
		} else if(shapeIdentifier == 2) {
			ROS_INFO("No Object Found");
		} else {
			ROS_INFO("ERROR");
		}
		//ROS_INFO("Shape (0 = Circle, 1 = Square, 2 = Rectangle) -> [%f]", shapeIdentifier);
		ros::spinOnce();
		loop_rate.sleep();

		// publish to the twist to the topic
		vel_pub_object.publish(velocityCommand);
	}

	return 0;
}
