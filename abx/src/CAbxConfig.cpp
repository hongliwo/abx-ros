/*
 * CAbx.cpp
 *
 * Created on: 2024-07-24
 * Author: henry@abx
 */

#include "ros/ros.h"
#include "CAbxConfig.h"

#define LOCAL_CONFIG_XML		ABX_CONFIG_XML

const AbxConfig abxDefConfig = 
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

CAbxConfig::CAbxConfig()
{
	loadConfig();
}

void CAbxConfig::loadConfig()
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

void CAbxConfig::saveConfig()
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

void CAbxConfig::restoreConfig()
{
	mConfig = abxDefConfig;

	saveConfig();
}

void CAbxConfig::generateConfig()
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
