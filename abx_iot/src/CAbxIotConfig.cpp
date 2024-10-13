/*
 * CAbxIot.cpp
 *
 * Created on: 2024-10-13
 * Author: henry@abx
 */

#include "ros/ros.h"
#include "CAbxIotConfig.h"

#define LOCAL_CONFIG_XML		ABX_IOT_CONFIG_XML

const AbxIotConfig abxIotDefConfig = 
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

CAbxIotConfig::CAbxIotConfig()
{
	loadConfig();
}

void CAbxIotConfig::loadConfig()
{
	std::ifstream ifs (LOCAL_CONFIG_XML);
	if (!ifs)
	{
		ROS_ERROR("Load abx iot config fail, now restore to default config");
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

void CAbxIotConfig::saveConfig()
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

void CAbxIotConfig::restoreConfig()
{
	mConfig = abxIotDefConfig;

	saveConfig();
}

void CAbxIotConfig::generateConfig()
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
