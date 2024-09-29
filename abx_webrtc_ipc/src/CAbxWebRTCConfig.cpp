/*
 * CAbxWebRTCConfig.cpp
 *
 * Created on: 2024-09-21
 * Author: henry@abx
 */

#include "ros/ros.h"
#include "CAbxWebRTCConfig.h"

#define LOCAL_CONFIG_XML		ABX_WEBRTC_CONFIG_XML

const AbxWebRTCConfig abxWebRTCDefConfig = 
{
	true,
	0,
	"",
	{
		false,
		0,
		""
	}
};

CAbxWebRTCConfig::CAbxWebRTCConfig()
{
	loadConfig();
}

void CAbxWebRTCConfig::loadConfig()
{
	std::ifstream ifs (LOCAL_CONFIG_XML);
	if (!ifs)
	{
		ROS_ERROR("Load abx webrtc config fail, now restore to default config");
		restoreConfig();
		return;
	}
	else
	{
		boost::archive::xml_iarchive oa (ifs);
		oa >> BOOST_SERIALIZATION_NVP (mConfig);
	}
	ifs.close();
}

void CAbxWebRTCConfig::saveConfig()
{
	std::ofstream ofs(LOCAL_CONFIG_XML);
	if (ofs)
	{
		boost::archive::xml_oarchive oa(ofs);
		oa << BOOST_SERIALIZATION_NVP(mConfig);
	}

	ofs.flush();
	ofs.close();
}

void CAbxWebRTCConfig::restoreConfig()
{
	mConfig = abxWebRTCDefConfig;

	saveConfig();
}

void CAbxWebRTCConfig::generateConfig()
{
	std::ofstream ofs(LOCAL_CONFIG_XML);
	if (ofs)
	{
		boost::archive::xml_oarchive oa(ofs);
		oa << BOOST_SERIALIZATION_NVP(mConfig);
	}

	ofs.flush();
	ofs.close();
}

// EOF
