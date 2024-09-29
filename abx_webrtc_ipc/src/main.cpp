/*
 * main.cpp
 *
 * Created on: 2024-09-21
 * Author: henry@abx.com
 */

#include <ros/ros.h>
#include <string>

#include "CAbxWebRTC.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "abx_webrtc");

	CAbxWebRTC AbxWebRTC;

	ros::spin();

	return 0;
}
