/*
 * main.cpp
 *
 * Created on: 2024-07-25
 * Author: henry@abx.com
 */

#include <ros/ros.h>
#include <string>

#include "CAbxCamera.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "abx_camera");

	CAbxCamera AbxCamera;

	ros::spin();

	return 0;
}
