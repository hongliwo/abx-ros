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

#include <iomanip>
#include <filesystem>
#include <cstdint>

#include "CAbxWebRTC.h"

extern PSampleConfiguration gSampleConfiguration;

using namespace std;
using namespace ros;

class FrameWriter {
private:
    bool firstKeyFrameDetected = false;
    int frameCount = 0;
    const int maxFrames = 1500;
    const std::string outputDir = "h264SampleFrames-hk-cbr";

public:
    FrameWriter() {
        mkdir(outputDir.c_str(), 0777);
    }

    void processFrame(const uint8_t* data, size_t size, bool isKeyFrame) {
        if (frameCount >= maxFrames) {
            return;
        }

        if (!firstKeyFrameDetected && isKeyFrame) {
            firstKeyFrameDetected = true;
        }

        if (firstKeyFrameDetected) {
            writeFrame(data, size);
        }
    }

private:
    void writeFrame(const uint8_t* data, size_t size) {
        frameCount++;
        std::stringstream ss;
        ss << outputDir << "/frame-"
           << std::setw(4) << std::setfill('0') << frameCount
           << ".h264";
        std::string filename = ss.str();

        std::ofstream file(filename.c_str(), std::ios::binary);
        if (file.is_open()) {
            file.write(reinterpret_cast<const char*>(data), size);
            file.close();
            std::cout << "Written frame " << frameCount << " to " << filename << std::endl;
        } else {
            std::cerr << "Failed to open file: " << filename << std::endl;
        }
    }
};

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
	mAbxMakeKeyFrameCli = mNh.serviceClient<std_srvs::Trigger>("abx_makeKeyFrame");

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
	//ROS_INFO("abxVideoSubCallback seq: %d",  msg->seq);
	//ROS_INFO_STREAM("Current stamp: " << msg->stamp);

    UINT32 i;
    UINT32 fileIndex = 0, frameSize = msg->size;
    Frame frame;
    STATUS status;
    RtcEncoderStats encoderStats;
    MEMSET(&encoderStats, 0x00, SIZEOF(RtcEncoderStats));
	static uint32_t oldStreamingSessionCount = 0;
	static bool makeKeyFrame = false;
	bool allSessionReady = true;

	static FrameWriter writer;
    //writer.processFrame(msg->data.data(), msg->size, msg->isKeyFrame);

	if (gSampleConfiguration != NULL)
	{
		//ROS_WARN("gSampleConfiguration != NULL, gSampleConfiguration->streamingSessionCount:%d", gSampleConfiguration->streamingSessionCount);

		if (gSampleConfiguration->streamingSessionCount != oldStreamingSessionCount)
		{
			makeKeyFrame = true;
			oldStreamingSessionCount = gSampleConfiguration->streamingSessionCount;
		}

		// Re-alloc if needed
        if (frameSize > gSampleConfiguration->videoBufferSize) {
            gSampleConfiguration->pVideoFrameBuffer = (PBYTE) MEMREALLOC(gSampleConfiguration->pVideoFrameBuffer, frameSize);
            gSampleConfiguration->videoBufferSize = frameSize;
        }

        frame.frameData = gSampleConfiguration->pVideoFrameBuffer;
        frame.size = frameSize;

		memcpy(frame.frameData, msg->data.data(), msg->size);

        // based on bitrate of samples/h264SampleFrames/frame-*
        encoderStats.width = 640;
        encoderStats.height = 480;
        encoderStats.targetBitrate = 262000;
        frame.presentationTs = msg->seq * SAMPLE_VIDEO_FRAME_DURATION;
        MUTEX_LOCK(gSampleConfiguration->streamingSessionListReadLock);
        for (i = 0; i < gSampleConfiguration->streamingSessionCount; ++i) {
            status = writeFrame(gSampleConfiguration->sampleStreamingSessionList[i]->pVideoRtcRtpTransceiver, &frame);
            if (gSampleConfiguration->sampleStreamingSessionList[i]->firstFrame && status == STATUS_SUCCESS) {
                PROFILE_WITH_START_TIME(gSampleConfiguration->sampleStreamingSessionList[i]->offerReceiveTime, "Time to first frame");
                gSampleConfiguration->sampleStreamingSessionList[i]->firstFrame = FALSE;
            }
            encoderStats.encodeTimeMsec = 4; // update encode time to an arbitrary number to demonstrate stats update
            updateEncoderStats(gSampleConfiguration->sampleStreamingSessionList[i]->pVideoRtcRtpTransceiver, &encoderStats);
            if (status != STATUS_SRTP_NOT_READY_YET) {
                if (status != STATUS_SUCCESS) {
                    DLOGV("writeFrame() failed with 0x%08x", status);
                } else {
                    DLOGV("writeFrame() video success with %d", msg->size);
				}
            } else {
                // Reset file index to ensure first frame sent upon SRTP ready is a key frame.
                fileIndex = 0;
				allSessionReady = false;
				ROS_WARN("!!! STATUS_SRTP_NOT_READY_YET");
            }
        }
		//ROS_INFO("gSampleConfiguration->streamingSessionCount:%d, allSessionReady:%d, makeKeyFrame:%d, isKeyFrame:%d, size:%d", 
		//		gSampleConfiguration->streamingSessionCount, allSessionReady, makeKeyFrame, msg->isKeyFrame, msg->size);
		if (gSampleConfiguration->streamingSessionCount > 0
				&& allSessionReady
				&& makeKeyFrame)
		{
			ROS_WARN("1 AbxMakeKeyFrame call");
			std_srvs::Trigger srv;
			if (mAbxMakeKeyFrameCli.call(srv))
			{
				ROS_INFO("srv response success: %d, message:%s", srv.response.success, mStr.data());
			}
			ROS_WARN("2 AbxMakeKeyFrame call");
			makeKeyFrame = false;
		}

        MUTEX_UNLOCK(gSampleConfiguration->streamingSessionListReadLock);
	}
}

void CAbxWebRTC::abxAudioSubCallback(const abx_msgs::AbxAudio::ConstPtr& msg)
{
	//ROS_INFO("abxAudioSubCallback seq: %d, size:%d",  msg->seq, msg->size);
	//ROS_INFO_STREAM("Current stamp: " << msg->stamp);

    UINT32 i;
    UINT32 fileIndex = 0, frameSize = msg->size;
    Frame frame;
    STATUS status;

	if (gSampleConfiguration != NULL)
	{
		// Re-alloc if needed
        if (frameSize > gSampleConfiguration->audioBufferSize) {
            gSampleConfiguration->pAudioFrameBuffer = (PBYTE) MEMREALLOC(gSampleConfiguration->pAudioFrameBuffer, frameSize);
            gSampleConfiguration->audioBufferSize = frameSize;
        }

        frame.frameData = gSampleConfiguration->pAudioFrameBuffer;
        frame.size = frameSize;

		memcpy(frame.frameData, msg->data.data(), msg->size);

        frame.presentationTs = msg->seq * SAMPLE_AUDIO_FRAME_DURATION;
        MUTEX_LOCK(gSampleConfiguration->streamingSessionListReadLock);
        for (i = 0; i < gSampleConfiguration->streamingSessionCount; ++i) {
            status = writeFrame(gSampleConfiguration->sampleStreamingSessionList[i]->pAudioRtcRtpTransceiver, &frame);
            if (status != STATUS_SRTP_NOT_READY_YET) {
                if (status != STATUS_SUCCESS) {
                    DLOGV("writeFrame() failed with 0x%08x", status);
                } else if (gSampleConfiguration->sampleStreamingSessionList[i]->firstFrame && status == STATUS_SUCCESS) {
                    PROFILE_WITH_START_TIME(gSampleConfiguration->sampleStreamingSessionList[i]->offerReceiveTime, "Time to first frame");
                    gSampleConfiguration->sampleStreamingSessionList[i]->firstFrame = FALSE;
                } else {
                    DLOGV("writeFrame() audio success with %d", msg->size);
				}
            } else {
                // Reset file index to stay in sync with video frames.
                fileIndex = 0;
            }
        }
        MUTEX_UNLOCK(gSampleConfiguration->streamingSessionListReadLock);
	}
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
