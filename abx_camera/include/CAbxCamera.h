/*
 * CAbxCamera.h
 *
 * Created on: 2024-07-25
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

#include "CAbxCameraConfig.h"
#include "HCNetSDK.h"

#pragma pack(1)

typedef struct
{
    unsigned char startCode[3];
    unsigned char streamId[1];
}PackStreamStart;

typedef union
{
    unsigned short  length;
    unsigned char   byte[2];
} PackStreamLESize;

typedef struct
{
    PackStreamStart     packStart;
    PackStreamLESize    packLen;
    char                packInfo[2];
    unsigned char       stuffingLen;
} PackStreamHead;

#pragma pack()

class CAbxCamera
{
public:
	CAbxCamera();
	virtual ~CAbxCamera();

private:
	void abxSubCallback(const std_msgs::Header::ConstPtr& msg);

private:
	bool abxSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
	void abxThreadFunc();

private:
	void abxTimerCallback();

private:
	static void psDataCallback(LONG lRealHandle, DWORD dwDataType,BYTE *pPacketBuffer,DWORD nPacketSize, void* pUser);

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
	int mUserId;
	int mRealPlayHandle;
	int mChannel;
	int mVideoSize;
	char mVideoData[2*1024*1024];

private:
	CAbxCameraConfig mAbxConfig;
};

