/*
 * CAbx.cpp
 *
 * Created on: 2024-07-24
 * Author: henry@abx
 */

#include "ros/ros.h"
#include "CAbxCameraConfig.h"

#define LOCAL_CONFIG_XML		ABX_CAMERA_CONFIG_XML

const AbxCameraConfig abxDefConfig = 
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

CAbxCameraConfig::CAbxCameraConfig()
{
	loadConfig();
}

void CAbxCameraConfig::loadConfig()
{
	std::ifstream ifs (LOCAL_CONFIG_XML);
	if (!ifs)
	{
		ROS_ERROR("Load abx config fail, now restore to default config");
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

void CAbxCameraConfig::saveConfig()
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

void CAbxCameraConfig::restoreConfig()
{
	mConfig = abxDefConfig;

	saveConfig();
}

void CAbxCameraConfig::generateConfig()
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
