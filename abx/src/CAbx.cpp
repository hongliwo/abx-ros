/*
 * CAbx.cpp
 *
 * Created on: 2024-07-24
 * Author: henry@abx
 */

#include <fstream>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <iostream>
#include <math.h>
#include "CAbx.h"

using namespace std;
using namespace ros;

CAbx::CAbx()
{
	mBool = false;
	mInt = 0;
	mStr = "";

	mAbxPub = mNh.advertise<std_msgs::Header>("abx", 10);

	mAbxSub = mNh.subscribe("abx", 10, &CAbx::abxSubCallback, this);
	
	mAbxSrv = mNh.advertiseService("abx", &CAbx::abxSrvCallback, this);

	mAbxCli = mNh.serviceClient<std_srvs::SetBool>("abx");

	mAbxThread = new boost::thread(boost::bind(&CAbx::abxThreadFunc, this));

	mAbxTimer = mNh.createTimer(ros::Duration(1), boost::bind(&CAbx::abxTimerCallback, this));
}

CAbx::~CAbx()
{
	if (mAbxThread)
	{
		delete mAbxThread;
	}
}

void CAbx::abxSubCallback(const std_msgs::Header::ConstPtr& msg)
{
	ROS_INFO("Sub header freame_id: %s, seq: %d", msg->frame_id.data(), msg->seq);
	ROS_INFO_STREAM("Current stamp: " << msg->stamp);
}

bool CAbx::abxSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	ROS_INFO("Service Callback req.data: %d", req.data);

	res.success = true;
	res.message = "successful";

	return true;
}

void CAbx::abxThreadFunc()
{
	Rate rate(1);

	while (ok())
	{
		std_msgs::Header msg;
		msg.seq = mInt++;
		msg.stamp = ros::Time::now();
		msg.frame_id = "abx";
		mAbxPub.publish(msg);

		std_srvs::SetBool srv;
		mBool = !mBool;
		srv.request.data = mBool;
		if (mAbxCli.call(srv))
		{
			mStr = srv.response.message;
			ROS_INFO("srv response success: %d, message:%s", srv.response.success, mStr.data());
		}

		rate.sleep();
	}
}

void CAbx::abxTimerCallback()
{
	ROS_WARN("Timer Callback");
}


// EOF
