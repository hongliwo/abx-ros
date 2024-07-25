/*
 * CAbxCamera.cpp
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
#include "CAbxCamera.h"

using namespace std;
using namespace ros;

CAbxCamera::CAbxCamera()
{
	mBool = false;
	mInt = 0;
	mStr = "";
	mChannel = 1;

	NET_DVR_Init();
    NET_DVR_SetLogToFile(3, "/opt/abx/log");

	mAbxPub = mNh.advertise<std_msgs::Header>("abx", 10);

	mAbxSub = mNh.subscribe("abx", 10, &CAbxCamera::abxSubCallback, this);
	
	mAbxSrv = mNh.advertiseService("abx", &CAbxCamera::abxSrvCallback, this);

	mAbxCli = mNh.serviceClient<std_srvs::SetBool>("abx");

	mAbxThread = new boost::thread(boost::bind(&CAbxCamera::abxThreadFunc, this));

	mAbxTimer = mNh.createTimer(ros::Duration(1), boost::bind(&CAbxCamera::abxTimerCallback, this));
}

CAbxCamera::~CAbxCamera()
{
	NET_DVR_Cleanup();

	if (mAbxThread)
	{
		delete mAbxThread;
	}
}

void CAbxCamera::abxSubCallback(const std_msgs::Header::ConstPtr& msg)
{
	ROS_INFO("Sub header freame_id: %s, seq: %d", msg->frame_id.data(), msg->seq);
	ROS_INFO_STREAM("Current stamp: " << msg->stamp);
}

bool CAbxCamera::abxSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	ROS_INFO("Service Callback req.data: %d", req.data);

	res.success = true;
	res.message = "successful";

	return true;
}

void CAbxCamera::abxThreadFunc()
{
	NET_DVR_DEVICEINFO_V30 struDeviceInfo;
	
	mUserId = NET_DVR_Login_V30("192.168.0.135", 8000, "admin", "1131HRuler1204", &struDeviceInfo);
	if (mUserId >= 0)
    {
		ROS_INFO("NET_DVR_Login_V30 successful");

        NET_DVR_PREVIEWINFO struPreviewInfo = {0};
        struPreviewInfo.lChannel = mChannel;
        struPreviewInfo.dwStreamType = 0;
        struPreviewInfo.dwLinkMode = 0;
        struPreviewInfo.bBlocked = 1;
        struPreviewInfo.bPassbackRecord  = 1;
        mRealPlayHandle = NET_DVR_RealPlay_V40(mUserId, &struPreviewInfo, &CAbxCamera::psDataCallback, this);
        if (mRealPlayHandle >= 0)
        {
            ROS_ERROR("[GetStream]---RealPlay %s:%d success,%d \n", "192.168.0.135", mChannel, NET_DVR_GetLastError());
        }
        else
        {
            ROS_ERROR("[GetStream]---RealPlay %s:%d failed, error = %d\n", "192.168.0.135", mChannel, NET_DVR_GetLastError());
        }
    }
    else
    {
        ROS_ERROR("[GetStream]---Login %s failed, error = %d\n", "192.168.0.135", NET_DVR_GetLastError());
    }

	Rate rate(1);

	while (ok())
	{
#if 0
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
#endif
		rate.sleep();
	}

	NET_DVR_StopRealPlay(mRealPlayHandle);
}

void CAbxCamera::abxTimerCallback()
{
	//ROS_WARN("Timer Callback");
}

//typedef void (CALLBACK *REALDATACALLBACK) (LONG lPlayHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* pUser);
void CAbxCamera::psDataCallback(LONG lRealHandle, DWORD dwDataType,BYTE *pPacketBuffer,DWORD nPacketSize, void* pUser)
{
	//ROS_INFO("dwDataType:%d, %s:%d", dwDataType, __func__, __LINE__);

    CAbxCamera *pThis = (CAbxCamera *)pUser;

    if (NET_DVR_STREAMDATA == dwDataType)
    {
        PackStreamStart *psStart = (PackStreamStart *)pPacketBuffer;
		//ROS_INFO("streamId:%#x, nPacketSize:%d, %s:%d", psStart->streamId[0], nPacketSize, __func__, __LINE__);

        if (psStart->streamId[0] == 0xba)
        {
		ROS_INFO("streamId:%#x, nPacketSize:%d, %s:%d", psStart->streamId[0], nPacketSize, __func__, __LINE__);
            if (pThis->mVideoSize > 0)
            {
#if 0
                bsp_msgs::BspVideo videoMsg;
                videoMsg.data.resize( pThis->mVideoSize );
                videoMsg.seq = seq++;
                videoMsg.size = pThis->mVideoSize;
#endif
                //ROS_WARN("pThis->mVideoSize:%d videoMsg.seq:%d ### %s:%d", pThis->mVideoSize, videoMsg.seq, __func__, __LINE__);
#if 0
				memcpy( &videoMsg.data[0], pThis->mVideoData, pThis->mVideoSize );
                pThis->mBspVideoPub.publish(videoMsg);
                //ROS_INFO("publish frame, size:%d\n", pThis->mVideoSize);
#endif
			}

			pThis->mVideoSize = 0;
		}
		else if (psStart->streamId[0] == 0xe0)
		{
			PackStreamHead *psHead = (PackStreamHead *)pPacketBuffer;
			PackStreamLESize psLen;
            psLen.byte[0] = psHead->packLen.byte[1];
            psLen.byte[1] = psHead->packLen.byte[0];
            int payloadLen = psLen.length - 3 - psHead->stuffingLen;
            int payloadOffset = sizeof(PackStreamHead) + psHead->stuffingLen;
			
			ROS_INFO("streamId:%#x, nPacketSize:%d, payloadLen:%d, %s:%d", 
					psStart->streamId[0], nPacketSize, payloadLen, __func__, __LINE__);

            //ROS_ERROR("sizeof(PackStreamHead):%d, psHead->stuffingLen:%d, sizeof(PackStreamStart):%d, sizeof(PackStreamLESize):%d",
            //      sizeof(PackStreamHead), psHead->stuffingLen, sizeof(PackStreamStart), sizeof(PackStreamLESize));

            //ROS_INFO("psLen.length:%d,payloadLen:%d,payloadOffset:%d,pThis->mVideoSize:%d ### %s:%d", psLen.length, payloadLen,payloadOffset,pThis-  >mVideoSize,__func__, __LINE__);

            //memcpy(&pThis->mVideoData[pThis->mVideoSize], &pPacketBuffer[payloadOffset], payloadLen);
            //pThis->mVideoSize += payloadLen;
        }
		else if (psStart->streamId[0] == 0xc0)
		{
			PackStreamHead *psHead = (PackStreamHead *)pPacketBuffer;
			PackStreamLESize psLen;
            psLen.byte[0] = psHead->packLen.byte[1];
            psLen.byte[1] = psHead->packLen.byte[0];
            int payloadLen = psLen.length - 3 - psHead->stuffingLen;
            int payloadOffset = sizeof(PackStreamHead) + psHead->stuffingLen;
			
			ROS_INFO("streamId:%#x, nPacketSize:%d, payloadLen:%d, %s:%d", 
					psStart->streamId[0], nPacketSize, payloadLen, __func__, __LINE__);
		}
		else
        {
            // other
        }
    }
}


// EOF
