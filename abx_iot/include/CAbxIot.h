/*
 * CAbxIot.h
 *
 * Created on: 2024-10-13
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


#include <aws/crt/Api.h>
#include <aws/crt/UUID.h>
#include <aws/crt/mqtt/Mqtt5Packets.h>
#include <aws/iot/Mqtt5Client.h>

#include <thread>


#include "CAbxIotConfig.h"

class CAbxIot
{
public:
	CAbxIot();
	virtual ~CAbxIot();

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
	Aws::Crt::ApiHandle apiHandle;

private:
	CAbxIotConfig mAbxConfig;
};

