/*
 * CAbxWebRTC.cpp
 *
 * Created on: 2024-09-21
 * Author: henry@abx
 */

#include <fstream>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <iostream>
#include <math.h>
#include "CAbxWebRTC.h"

using namespace std;
using namespace ros;

CAbxWebRTC::CAbxWebRTC()
{
	mBool = false;
	mInt = 0;
	mStr = "";

	mAbxPub = mNh.advertise<std_msgs::Header>("abx", 10);

	mAbxVideoSub = mNh.subscribe("abx_video", 50, &CAbxWebRTC::abxVideoSubCallback, this);
	mAbxAudioSub = mNh.subscribe("abx_audio", 50, &CAbxWebRTC::abxAudioSubCallback, this);
	
	mAbxSrv = mNh.advertiseService("abx_webrtc", &CAbxWebRTC::abxSrvCallback, this);

	mAbxCli = mNh.serviceClient<std_srvs::SetBool>("abx");

	mAbxThread = new boost::thread(boost::bind(&CAbxWebRTC::abxThreadFunc, this));

	mAbxTimer = mNh.createTimer(ros::Duration(1), boost::bind(&CAbxWebRTC::abxTimerCallback, this));
}

CAbxWebRTC::~CAbxWebRTC()
{
	sigintHandler(2);
	
	if (mAbxThread)
	{
		delete mAbxThread;
	}

}

void CAbxWebRTC::abxVideoSubCallback(const abx_msgs::AbxVideo::ConstPtr& msg)
{
	ROS_INFO("abxVideoSubCallback seq: %d",  msg->seq);
	//ROS_INFO_STREAM("Current stamp: " << msg->stamp);
}

void CAbxWebRTC::abxAudioSubCallback(const abx_msgs::AbxAudio::ConstPtr& msg)
{
	ROS_INFO("abxAudioSubCallback seq: %d",  msg->seq);
	//ROS_INFO_STREAM("Current stamp: " << msg->stamp);
}

bool CAbxWebRTC::abxSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	ROS_INFO("Service Callback req.data: %d", req.data);

	res.success = true;
	res.message = "successful";

	return true;
}

void CAbxWebRTC::abxThreadFunc()
{
	char* args[] = {"kvsWebRTCClientMaster", "demo-channel"};
	int length = sizeof(args) / sizeof(args[0]);
	kvsWebRTCClientMaster(length, args);



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

void CAbxWebRTC::abxTimerCallback()
{
	ROS_WARN("Timer Callback");
}


// EOF
