/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * This file has been adapted from setVehicleVelocity.cpp
 * authored by Mario Prats and Javier Perez
 *
 * Contributors:
 *     Juan Carlos García
 */ 

#include <iostream>
#include <stdlib.h>
#include <string.h>

//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#define TOPIC  "/dataNavigator_G500RAUVI"		//shipweck
//#define TOPIC  "/dataNavigator"				//CIRS

using namespace std;


class Leap2Uwsim {

public:
	Leap2Uwsim();

private:
	void leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped);

	ros::NodeHandle nh_;

	float initPosition[3];
	float currentPosition[3];
	float initOrientation[4];
	float currentOrientation[4];
	ros::Publisher vel_pub_;
	ros::Subscriber leap_sub_;
};

Leap2Uwsim::Leap2Uwsim() {
	//initializing values
	initPosition[0] = 0;
	initPosition[1] = 0;
	initPosition[2] = 0;
	initOrientation[0] = 0;
	initOrientation[1] = 0;
	initOrientation[2] = 0;
	initOrientation[3] = 0;

	//publisher and subscriber initialization
	vel_pub_ = nh_.advertise<nav_msgs::Odometry>(TOPIC,1);
	leap_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("leap_tracker/pose_stamped_out", 1, &Leap2Uwsim::leapCallback, this);
}


void Leap2Uwsim::leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped) {
	int num;

	//Initial user hand position
	if ((initPosition[0] == 0) and (initPosition[1] == 0) and (initPosition[2] == 0)) {
		initPosition[0] = posstamped->pose.position.x;
		initPosition[1] = posstamped->pose.position.y;
		initPosition[2] = posstamped->pose.position.z;
		initOrientation[0] = posstamped->pose.orientation.x;
		initOrientation[1] = posstamped->pose.orientation.y;
		initOrientation[2] = posstamped->pose.orientation.z;
		initOrientation[3] = posstamped->pose.orientation.w;
		cout << "Starting hand position: (" << \
			initPosition[0] << "," << initPosition[1] << "," << initPosition[2] << ")" << endl;
		cout << "Press Enter to continue... ";
		num = getchar();		
	}

	nav_msgs::Odometry odom;
	//Keep all with 0. We send velocities, not position.
	odom.pose.pose.position.x=0.0;
	odom.pose.pose.position.y=0.0;
	odom.pose.pose.position.z=0.0;
	odom.pose.pose.orientation.x=0.0;
	odom.pose.pose.orientation.y=0.0;
	odom.pose.pose.orientation.z=0.0;
	odom.pose.pose.orientation.w=1;

	cout << "Current hand position: (";
	//X-axis
	currentPosition[0] = posstamped->pose.position.x - abs(initPosition[0]);
	cout << currentPosition[0] << ",";
	if ((currentPosition[0] >= -15.0) and (currentPosition[0] <= 15.0))
		currentPosition[0] = 0.00;
	else
		currentPosition[0] = (currentPosition[0] < 0 ? -0.05 : 0.05);
	//Y-axis
	currentPosition[1] = posstamped->pose.position.y - initPosition[1];
	cout << currentPosition[1] << ",";
	if (currentPosition[1] <= 10)
		currentPosition[1] = 0.00;
	else
		currentPosition[1] = (currentPosition[1] < initPosition[1] ? 0.05 : -0.05);
	//Z-axis
	currentPosition[2] = posstamped->pose.position.z - abs(initPosition[2]);
	cout << currentPosition[2] << ")" << endl;
	if ((currentPosition[2] >= -15.0) and (currentPosition[2] <= 15.0))
		currentPosition[2] = 0.00;
	else
		currentPosition[2] = (currentPosition[2] < 0 ? 0.05 : -0.05);

	//TODO
	currentOrientation[0] = (posstamped->pose.orientation.x - initPosition[0] ? 0.05 : -0.05);
	currentOrientation[1] = (posstamped->pose.orientation.y - initPosition[1] ? 0.05 : -0.05);
	currentOrientation[2] = (posstamped->pose.orientation.z - initPosition[2] ? 0.05 : -0.05);

	odom.twist.twist.linear.x = currentPosition[2]; 
	odom.twist.twist.linear.y = currentPosition[0];
	odom.twist.twist.linear.z = currentPosition[1];
	odom.twist.twist.angular.x = 0; //roll;
	odom.twist.twist.angular.y = 0; //pitch;
	odom.twist.twist.angular.z = 0; //yaw;
	for (int i=0; i<36; i++) {
		odom.twist.covariance[i]=0;
		odom.pose.covariance[i]=0;
	}

	vel_pub_.publish(odom);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "leap2uwsim");
  Leap2Uwsim leap2uwsim_control;
  ros::spin();
}
