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
	mVideoSize = 0;
	mIsKeyFrame = false;

	NET_DVR_Init();
    NET_DVR_SetLogToFile(3, "/opt/abx/log");

	mAbxPub = mNh.advertise<std_msgs::Header>("abx", 10);
	mAbxVideoPub = mNh.advertise<abx_msgs::AbxVideo>("abx_video", 50);
	mAbxAudioPub = mNh.advertise<abx_msgs::AbxAudio>("abx_audio", 50);

	mAbxSub = mNh.subscribe("abx", 10, &CAbxCamera::abxSubCallback, this);
	
	mAbxSrv = mNh.advertiseService("abx", &CAbxCamera::abxSrvCallback, this);
	mAbxMakeKeyFrameSrv = mNh.advertiseService("abx_makeKeyFrame", &CAbxCamera::abxMakeKeyFrameSrvCallback, this);

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

bool CAbxCamera::abxMakeKeyFrameSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	ROS_INFO("abxMakeKeyFrame Service Callbackd");

	if (mUserId >= 0) 
	{
		BOOL ret = NET_DVR_MakeKeyFrame(mUserId, mChannel);
		ROS_WARN("########### NET_DVR_MakeKeyFrame:%d, %d", ret, NET_DVR_GetLastError());
		ret = NET_DVR_MakeKeyFrameSub(mUserId, mChannel);
		ROS_WARN("########### NET_DVR_MakeKeyFrameSub:%d, %d", ret, NET_DVR_GetLastError());
	}
	
	res.success = true;
	res.message = "successful";

	return true;
}

void CAbxCamera::abxThreadFunc()
{
	NET_DVR_DEVICEINFO_V30 struDeviceInfo;
	
	mUserId = NET_DVR_Login_V30("192.168.0.146", 8000, "abx", "a12345678", &struDeviceInfo);
	if (mUserId >= 0)
    {
		ROS_INFO("NET_DVR_Login_V30 successful");

		// https://open.hikvision.com/hardware/structures/NET_DVR_PREVIEWINFO.html
        NET_DVR_PREVIEWINFO struPreviewInfo = {0};
        struPreviewInfo.lChannel = mChannel;
        struPreviewInfo.dwStreamType = 0; // 0;
        struPreviewInfo.dwLinkMode = 0;
        struPreviewInfo.bBlocked = 1;
        struPreviewInfo.bPassbackRecord  = 1;
        mRealPlayHandle = NET_DVR_RealPlay_V40(mUserId, &struPreviewInfo, &CAbxCamera::psDataCallback, this);
        if (mRealPlayHandle >= 0)
        {
            ROS_INFO("[GetStream]---RealPlay %s:%d success,%d \n", "192.168.0.135", mChannel, NET_DVR_GetLastError());
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

	static uint32_t videoSeq = 0;
	static uint32_t audioSeq = 0;
    CAbxCamera *pThis = (CAbxCamera *)pUser;


	// https://blog.yasking.org/a/hikvision-rtp-ps-stream-parser.html
    if (NET_DVR_STREAMDATA == dwDataType)
    {
        PackStreamStart *psStart = (PackStreamStart *)pPacketBuffer;
		//ROS_INFO("streamId:%#x, nPacketSize:%d, %s:%d", psStart->streamId[0], nPacketSize, __func__, __LINE__);

		// https://blog.csdn.net/aflyeaglenku/article/details/107106197
        if (psStart->streamId[0] == 0xba)
        {
			PsHeader *psHeader = (PsHeader *)pPacketBuffer;
			int extended_size = psHeader->extended_size & 0x7;
			//ROS_INFO("extended_size:%d, sizeof(PsHeader):%d", extended_size, sizeof(PsHeader));

			BYTE *p = &pPacketBuffer[sizeof(PsHeader) + extended_size];
			uint32_t head_code = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
			
			if (PSM_START_CODE == head_code)
			{
				//ROS_INFO("PSM_START_CODE");
				ProgramStreamMap *psm = (ProgramStreamMap *)p;
				uint16_t program_stream_map_length = (psm->psm_length[0] << 8) | psm->psm_length[1];
				uint16_t program_stream_info_length = (psm->psi_length[0] << 8) | psm->psi_length[1];
				//ROS_INFO("PSM_START_CODE, psm_length:%x, psi_length:%x",
				//		program_stream_map_length, program_stream_info_length);

				BYTE *p_element_stream_map_length = (BYTE *)&p[sizeof(ProgramStreamMap) + program_stream_info_length];
				uint16_t element_stream_map_length = p_element_stream_map_length[0] << 8 | p_element_stream_map_length[1];
				//ROS_INFO("element_stream_map_length:%x, %x,%x,%x,%x", 
				//		element_stream_map_length,
				//		p_element_stream_map_length[0], p_element_stream_map_length[1],
				//		p_element_stream_map_length[2], p_element_stream_map_length[3]
				//		);
				MediaStreamInfo *videoStreamInfo = (MediaStreamInfo *)&p[sizeof(ProgramStreamMap) + program_stream_info_length + 2];
				uint16_t videoExtendedLength = videoStreamInfo->extended_length[0] << 8 | videoStreamInfo->extended_length[1];
				//ROS_INFO("video stream_type:%x, stream_id:%x, extended_length:%x", 
				//		videoStreamInfo->stream_type, videoStreamInfo->stream_id, videoExtendedLength);
				pThis->mVideoFormat = videoStreamInfo->stream_type == 0x1b ? abx_msgs::AbxVideo::VIDEO_FORMAT_H264 : abx_msgs::AbxVideo::VIDEO_FORMAT_H265;
				MediaStreamInfo *audioStreamInfo = (MediaStreamInfo *)&p[sizeof(ProgramStreamMap) + program_stream_info_length + 2 + sizeof(MediaStreamInfo) + videoExtendedLength];
				uint16_t audioExtendedLength = audioStreamInfo->extended_length[0] << 8 | audioStreamInfo->extended_length[1];
				//ROS_INFO("audio stream_type:%x, stream_id:%x, extended_length:%x", 
				//		audioStreamInfo->stream_type, audioStreamInfo->stream_id, audioExtendedLength);
				pThis->mAudioFormat = audioStreamInfo->stream_type == 0x90 ? abx_msgs::AbxAudio::AUDIO_FORMAT_G711alaw : abx_msgs::AbxAudio::AUDIO_FORMAT_AAC;
			}
			else if (SYSTEM_HEADER_START_CODE == head_code)
			{
				//ROS_INFO("SYSTEM_HEADER_START_CODE");
			}
			else
			{
			}

			ROS_INFO("streamId:%#x, nPacketSize:%d, %s:%d", psStart->streamId[0], nPacketSize, __func__, __LINE__);
            //ROS_INFO("%x,%x,%x,%x, %x", pPacketBuffer[0], pPacketBuffer[1],pPacketBuffer[2],pPacketBuffer[3], pPacketBuffer[13]);
            //ROS_INFO("%x,%x,%x,%x, %x, %x", pPacketBuffer[20], pPacketBuffer[21],pPacketBuffer[22],pPacketBuffer[23], pPacketBuffer[24], pPacketBuffer[25]);
            //ROS_INFO("%x,%x,%x,%x, %x, %x", pPacketBuffer[32], pPacketBuffer[33],pPacketBuffer[34],pPacketBuffer[35], pPacketBuffer[36], pPacketBuffer[37]);

			if (pThis->mVideoSize > 0)
            {
				abx_msgs::AbxVideo videoMsg;
                videoMsg.data.resize( pThis->mVideoSize );
                videoMsg.seq = videoSeq++;
                videoMsg.size = pThis->mVideoSize;
				videoMsg.videoFormat = pThis->mVideoFormat;
				videoMsg.isKeyFrame = pThis->mIsKeyFrame;
                //ROS_WARN("pThis->mVideoSize:%d videoMsg.seq:%d ### %s:%d", pThis->mVideoSize, videoMsg.seq, __func__, __LINE__);
				memcpy( &videoMsg.data[0], pThis->mVideoData, pThis->mVideoSize );
                pThis->mAbxVideoPub.publish(videoMsg);
                ROS_WARN("publish video frame, seq:%d, size:%d, videoFormat:%s, isKeyFrame:%d", 
						videoMsg.seq, videoMsg.size, videoMsg.videoFormat == abx_msgs::AbxVideo::VIDEO_FORMAT_H264 ? "H264" : "H265", 
						videoMsg.isKeyFrame);
				if (videoMsg.isKeyFrame)
				{
					printf("\n");
				}
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
			

			ROS_INFO("streamId:%#x, nPacketSize:%d, payloadLen:%d, packInfo:%x,%x, type:%x, %s:%d", 
					psStart->streamId[0], nPacketSize, payloadLen, psHead->packInfo[0], psHead->packInfo[1],
					pPacketBuffer[payloadOffset + 4],
					__func__, __LINE__);

			uint8_t typeValue = pPacketBuffer[payloadOffset + 4];
			
			if (psHead->packInfo[0] == 0x8c)
			{
				if (abx_msgs::AbxVideo::VIDEO_FORMAT_H265 == pThis->mVideoFormat)
				{
					uint8_t naluType = (typeValue >> 1) & 0x3f;
					switch (naluType)
					{
						case H265_NAL_VPS:
						case H265_NAL_SPS:
						case H265_NAL_PPS:
						case H265_NAL_IDR_W_RADL:
						case H265_NAL_IDR_N_LP:
							pThis->mIsKeyFrame = true;
							break;
						case H265_NAL_TRAIL_R:
							pThis->mIsKeyFrame = false;
							break;
						default:
							break;
					}
				}
				else if (abx_msgs::AbxVideo::VIDEO_FORMAT_H264 == pThis->mVideoFormat)
				{
					uint8_t naluType = typeValue & 0x1f;
					switch (naluType)
					{
						case H264_NAL_SPS:
						case H264_NAL_PPS:
						case H264_NAL_SEI:
						case H264_NAL_IDR:
							pThis->mIsKeyFrame = true;
							break;
						case H264_NAL_P:
							pThis->mIsKeyFrame = false;
						default:
							break;
					}
				}
			}

            //ROS_ERROR("sizeof(PackStreamHead):%d, psHead->stuffingLen:%d, sizeof(PackStreamStart):%d, sizeof(PackStreamLESize):%d",
            //      sizeof(PackStreamHead), psHead->stuffingLen, sizeof(PackStreamStart), sizeof(PackStreamLESize));

            //ROS_INFO("psLen.length:%d,payloadLen:%d,payloadOffset:%d,pThis->mVideoSize:%d ### %s:%d", psLen.length, payloadLen,payloadOffset,pThis-  >mVideoSize,__func__, __LINE__);

            memcpy(&pThis->mVideoData[pThis->mVideoSize], &pPacketBuffer[payloadOffset], payloadLen);
            pThis->mVideoSize += payloadLen;
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

			abx_msgs::AbxAudio audioMsg;
			audioMsg.data.resize( payloadLen );
			audioMsg.seq = audioSeq++;
			audioMsg.size = payloadLen;
			audioMsg.audioFormat = pThis->mAudioFormat;
			//ROS_WARN("pThis->mVideoSize:%d videoMsg.seq:%d ### %s:%d", pThis->mVideoSize, videoMsg.seq, __func__, __LINE__);
			memcpy( &audioMsg.data[0], &pPacketBuffer[payloadOffset], payloadLen );
			pThis->mAbxAudioPub.publish(audioMsg);
			ROS_WARN("publish audio frame, seq:%d, size:%d, audioFormat:%s", 
					audioMsg.seq, audioMsg.size, audioMsg.audioFormat == abx_msgs::AbxAudio::AUDIO_FORMAT_G711alaw ? "G711alaw" : "AAC");
		}
		else
        {
            // other
			ROS_INFO("streamId:%#x ???, nPacketSize:%d, %s:%d", psStart->streamId[0], nPacketSize, __func__, __LINE__);
        }
    }
}


// EOF
