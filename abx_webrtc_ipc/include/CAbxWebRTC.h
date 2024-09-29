/*
 * CAbxWebRTC.h
 *
 * Created on: 2024-09-21
 * Author: henry@abx
 */

#include <fstream>
#include <openssl/sha.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <std_msgs/Header.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <boost/thread/thread.hpp>

#include "abx_msgs/AbxVideo.h"
#include "abx_msgs/AbxAudio.h"

#include "CAbxWebRTCConfig.h"
#include "Samples.h"

class CAbxWebRTC
{
public:
	CAbxWebRTC();
	virtual ~CAbxWebRTC();

private:
	void abxVideoSubCallback(const abx_msgs::AbxVideo::ConstPtr& msg);
	void abxAudioSubCallback(const abx_msgs::AbxAudio::ConstPtr& msg);

private:
	bool abxSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
	void abxThreadFunc();

private:
	void abxTimerCallback();

private:
    ros::NodeHandle mNh;
	ros::NodeHandle mPh;

private:
	boost::thread* mAbxThread;

private:
	ros::Timer mAbxTimer;
	
private:
	ros::Publisher mAbxPub;

private:
	ros::Subscriber mAbxVideoSub;
	ros::Subscriber mAbxAudioSub;

private:
	ros::ServiceServer mAbxSrv;

private:
	ros::ServiceClient mAbxCli;
	ros::ServiceClient mAbxMakeKeyFrameCli;

private:
	bool mBool;
	uint32_t mInt;
	std::string mStr;

private:
	CAbxWebRTCConfig mAbxWebRTCConfig;
};

