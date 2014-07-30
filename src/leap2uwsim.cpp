/* 
 * Copyright (c) 2014 University of Jaume-I.
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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//#define TOPIC  "/dataNavigator_G500RAUVI"		//shipweck scene
#define TOPIC  "/dataNavigator"				//CIRS scene

using namespace std;


class Leap2Uwsim
{

public:
	Leap2Uwsim();

private:
	void leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped);

	ros::NodeHandle nh_;

	float initPosition[3];
	float currentPosition[3];
	float previousPosition[3];
	float initOrientation[4];
	float currentOrientation[4];
	ros::Publisher vel_pub_;
	ros::Subscriber leap_sub_;
	tf::Transform transform_init, transform_new;
	tf::StampedTransform transform;
	tf::Quaternion q_init, q_new;
	tf::TransformBroadcaster br;
	tf::TransformListener *listener;
};


Leap2Uwsim::Leap2Uwsim()
{
	//initializing values
	initPosition[0] = 0;
	initPosition[1] = 0;
	initPosition[2] = 0;
	previousPosition[0] = 0;
	previousPosition[1] = 0;
	previousPosition[2] = 0;
	initOrientation[0] = 0;
	initOrientation[1] = 0;
	initOrientation[2] = 0;
	initOrientation[3] = 0;

	listener = new (tf::TransformListener);

	//publisher and subscriber initialization
	vel_pub_ = nh_.advertise<nav_msgs::Odometry>(TOPIC,1);
	leap_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("leap_tracker/pose_stamped_out", 1, &Leap2Uwsim::leapCallback, this);
}


void Leap2Uwsim::leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped)
{
	int num;
	double roll, pitch, yaw;
	

	//Initial user hand position
	if ((initPosition[0] == 0) and (initPosition[1] == 0) and (initPosition[2] == 0))
	{
		initPosition[0] = posstamped->pose.position.x;
		initPosition[1] = posstamped->pose.position.y;
		initPosition[2] = posstamped->pose.position.z;
		//Save initial hand orientation
		initOrientation[0] = posstamped->pose.orientation.x;
		initOrientation[1] = posstamped->pose.orientation.y;
		initOrientation[2] = posstamped->pose.orientation.z;
		initOrientation[3] = posstamped->pose.orientation.w;
		//Set initial transform data
		transform_init.setOrigin(tf::Vector3(initPosition[0], initPosition[1], initPosition[2]));
//		transform_init.setOrigin(tf::Vector3(posstamped->pose.orientation.x, \
								posstamped->pose.orientation.y, posstamped->pose.orientation.z));
		q_init = tf::Quaternion(posstamped->pose.orientation.x, posstamped->pose.orientation.y, \
								posstamped->pose.orientation.z, posstamped->pose.orientation.w);
		transform_init.setRotation(q_init.normalize());

		cout << "Starting hand position: (" << initPosition[0] << "," << initPosition[1] \
			 << "," << initPosition[2] << " :: " << initOrientation[1] << ")" << endl;
		cout << "Press Enter to continue... ";
		num = getchar();		
	}
	br.sendTransform(tf::StampedTransform(transform_init, ros::Time::now(), "world", "init_pose"));


	nav_msgs::Odometry odom;
	//Keep all with 0. We send velocities, not position.
	odom.pose.pose.position.x=0.0;
	odom.pose.pose.position.y=0.0;
	odom.pose.pose.position.z=0.0;
	odom.pose.pose.orientation.x=0.0;
	odom.pose.pose.orientation.y=0.0;
	odom.pose.pose.orientation.z=0.0;
	odom.pose.pose.orientation.w=1;

	//cout << "Absolute hand position: (" << posstamped->pose.position.x << ", " << \
		posstamped->pose.position.y << ", " << posstamped->pose.position.z << "-" << \
		posstamped->pose.orientation.y << ")" << endl;
	//if the user don't move the hand, the robot keep the position
	if ((posstamped->pose.position.x == previousPosition[0]) and \
		(posstamped->pose.position.y == previousPosition[1]) and \
		(posstamped->pose.position.z == previousPosition[2]))
	{
		for (int i=0; i<3; i++)
			currentPosition[i] = 0.00;
		currentOrientation[1] = 0.00;
	}
	else
	{
		//store previous position
		previousPosition[0] = posstamped->pose.position.x;
		previousPosition[1] = posstamped->pose.position.y;
		previousPosition[2] = posstamped->pose.position.z;

		//cout << "Referenced hand position: (" ;
		//LeapMotion X-axis -> Robot Y-axis
		currentPosition[0] = posstamped->pose.position.x - abs(initPosition[0]);
		//cout << currentPosition[0] << ",";
		cout << "Robot movement(s): ";
		if ((currentPosition[0] >= -15.0) and (currentPosition[0] <= 15.0))
			currentPosition[0] = 0.00;
		else
		{
			currentPosition[0] = (currentPosition[0] < 0 ? -0.2 : 0.2);
			cout << " Y-Axis: " ;
			(currentPosition[0] < 0 ? cout << " left |" : cout << " right |");
		}

		//LeapMotion Y-axis -> Robot Z-axis
		currentPosition[1] = posstamped->pose.position.y - abs(initPosition[1]);
		//cout << currentPosition[1] << ",";
		//cout << posstamped->pose.position.y << ", " << currentPosition[1] << "," << endl;
		if (currentPosition[1] <= 20)
			currentPosition[1] = 0.00;
		else
		{
			currentPosition[1] = (currentPosition[1] < initPosition[1] ? 0.2 : -0.2);
			cout << " Z-Axis: " ;
			(currentPosition[1] < 0 ? cout << " up |" : cout << " down |");
		}


		//LeapMotion Z-axis -> Robot X-axis
		currentPosition[2] = posstamped->pose.position.z - abs(initPosition[2]);
		//cout << currentPosition[2] << ")" << endl;
		if ((currentPosition[2] >= -15.0) and (currentPosition[2] <= 15.0))
			currentPosition[2] = 0.00;
		else
		{
			currentPosition[2] = (currentPosition[2] < 0 ? 0.2 : -0.2);
			cout << " X-Axis: " ;
			(currentPosition[2] < 0 ? cout << " back |" : cout << " front |");
		}

		//Y-orientation
		transform_new.setOrigin(tf::Vector3(posstamped->pose.orientation.x, \
								posstamped->pose.orientation.y, posstamped->pose.orientation.z));
		q_new = tf::Quaternion(posstamped->pose.orientation.x, posstamped->pose.orientation.y, \
								posstamped->pose.orientation.z, posstamped->pose.orientation.w);
		transform_new.setRotation(q_new.normalize());
		br.sendTransform(tf::StampedTransform(transform_new, ros::Time::now(), "world", "new_pose"));

		try{
			listener->lookupTransform("/new_pose", "/init_pose", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		transform.getBasis().getRPY(roll, pitch, yaw);
		//cout << "roll = " << roll << " | pitch = " << pitch << " | yaw = " << yaw << endl;
		if ((roll >= -0.1) and (roll <= 0.1))
			currentOrientation[1] = 0.00;
		else
		{
			currentOrientation[1] = (roll < 0 ? -0.2 : 0.2);
			cout << " Yaw: " ;
			(roll < 0 ? cout << " left |" : cout << " right |");
		}
		cout << endl;
	}

	//Assign the calculated values into the publisher
	odom.twist.twist.linear.x = currentPosition[2]; 
	odom.twist.twist.linear.y = currentPosition[0];
	odom.twist.twist.linear.z = currentPosition[1];
	odom.twist.twist.angular.x = 0; //roll;
	odom.twist.twist.angular.y = 0; //pitch;
	odom.twist.twist.angular.z = currentOrientation[1]; //yaw
	for (int i=0; i<36; i++)
	{
		odom.twist.covariance[i]=0;
		odom.pose.covariance[i]=0;
	}

	vel_pub_.publish(odom);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "leap2uwsim");
  Leap2Uwsim leap2uwsim_control;
  ros::spin();
}
