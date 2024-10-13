/*
 * main.cpp
 *
 * Created on: 2024-10-13
 * Author: henry@abx.com
 */

#include <ros/ros.h>
#include <string>

#include "CAbxIot.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "abx_iot");

	CAbxIot AbxIot;

	ros::spin();

	return 0;
}
