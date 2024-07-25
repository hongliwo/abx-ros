/*
 * main.cpp
 *
 * Created on: 2024-07-24
 * Author: henry@abx.com
 */

#include <ros/ros.h>
#include <string>

#include "CAbx.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "abx");

	CAbx Abx;

	ros::spin();

	return 0;
}
