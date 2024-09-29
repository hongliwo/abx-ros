/*
 * CAbxCamera.h
 *
 * Created on: 2024-07-25
 * Author: henry@abx
 * https://www.jianshu.com/p/00a2ed58a77b
 * H264 67(SPS) 68(PPS) 65(IDR) / 41(P)
 * H265 40(VPS) 42(SPS) 44(PPS) 26(IDR_W_RADL) / 2(TSA_N)
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

#include "CAbxCameraConfig.h"
#include "HCNetSDK.h"

typedef uint8_t  byte;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

#define PACK_START_CODE          0x000001BA
#define SYSTEM_HEADER_START_CODE 0x000001BB
#define PSM_START_CODE           0x000001BC
#define PES_START_CODE_PREFIX    0x000001
#define VIDEO_STREAM_START       0xE0
#define AUDIO_STREAM_START       0xC0
#define PRIVATE_STREAM_1         0xBD

enum {
};

enum H264NALUnitType {
	H264_NAL_SPS    = 7,
    H264_NAL_PPS    = 8,
	H264_NAL_SEI    = 6,
	H264_NAL_IDR    = 5,
    H264_NAL_P      = 1,
};

enum H265NALUnitType {
    H265_NAL_TRAIL_N    = 0,
    H265_NAL_TRAIL_R    = 1,
    H265_NAL_TSA_N      = 2,
    H265_NAL_TSA_R      = 3,
    H265_NAL_STSA_N     = 4,
    H265_NAL_STSA_R     = 5,
    H265_NAL_RADL_N     = 6,
    H265_NAL_RADL_R     = 7,
    H265_NAL_RASL_N     = 8,
    H265_NAL_RASL_R     = 9,
    H265_NAL_BLA_W_LP   = 16,
    H265_NAL_BLA_W_RADL = 17,
    H265_NAL_BLA_N_LP   = 18,
    H265_NAL_IDR_W_RADL = 19,
    H265_NAL_IDR_N_LP   = 20,
    H265_NAL_CRA_NUT    = 21,
    H265_NAL_VPS        = 32,
    H265_NAL_SPS        = 33,
    H265_NAL_PPS        = 34,
    H265_NAL_AUD        = 35,
    H265_NAL_EOS_NUT    = 36,
    H265_NAL_EOB_NUT    = 37,
    H265_NAL_FD_NUT     = 38,
    H265_NAL_SEI_PREFIX = 39,
    H265_NAL_SEI_SUFFIX = 40,
};

#pragma pack(1)

typedef struct {
    uint32_t start_code;         // 0x000000BA
	uint8_t  reserved[9];
    uint8_t  extended_size; 
} PsHeader;

typedef struct {
    uint32_t start_code;         // 0x000000BC
	uint8_t psm_length[2];
	uint8_t reserved[2];
	uint8_t psi_length[2];
} ProgramStreamMap;

typedef struct {
	uint8_t stream_type;	// H264:0x1B H265:0x24 G711:0x90 AAC:0x0F
	uint8_t stream_id;		// Video:0xE0 Audio:0xC0
	uint8_t extended_length[2];
} MediaStreamInfo;


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

typedef struct {
    unsigned char start_code[4];  // 0x000001BA
    unsigned char system_clock_reference[6];
    unsigned char program_mux_rate[3];
} PS_PACK_HEADER;

typedef struct {
    unsigned char start_code[3];  // 0x000001
    unsigned char stream_id;
    unsigned short PES_packet_length;
    unsigned char flags[2];
    unsigned char PES_header_data_length;
} PES_HEADER;

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
	bool abxMakeKeyFrameSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

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
	ros::Publisher mAbxVideoPub;
	ros::Publisher mAbxAudioPub;

private:
	ros::Subscriber mAbxSub;

private:
	ros::ServiceServer mAbxSrv;
	ros::ServiceServer mAbxMakeKeyFrameSrv;

private:
	ros::ServiceClient mAbxCli;

private:
	bool mBool;
	uint32_t mInt;
	std::string mStr;
	int mUserId;
	int mRealPlayHandle;
	int mChannel;
	int mAudioFormat;	// PCM:0 G711alaw:1 G711ulaw:2 G726:3 AAC:4
	int mVideoFormat;	// H264:0 H265:1
	bool mIsKeyFrame;	
	int mVideoSize;
	char mVideoData[2*1024*1024];

private:
	CAbxCameraConfig mAbxConfig;
};

