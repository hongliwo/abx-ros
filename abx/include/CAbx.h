/*
 * CAbx.h
 *
 * Created on: 2024-07-24
 * Author: henry@abx
 */

#include <fstream>
#include <openssl/sha.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <std_msgs/Header.h>
#include <std_srvs/SetBool.h>
#include <boost/thread/thread.hpp>

#include "CAbxConfig.h"

class CAbx
{
public:
	CAbx();
	virtual ~CAbx();

private:
	void abxSubCallback(const std_msgs::Header::ConstPtr& msg);

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
	ros::Subscriber mAbxSub;

private:
	ros::ServiceServer mAbxSrv;

private:
	ros::ServiceClient mAbxCli;

private:
	bool mBool;
	uint32_t mInt;
	std::string mStr;

private:
	CAbxConfig mAbxConfig;
};

