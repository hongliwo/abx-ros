/*
 * CAbxWebRTCConfig.h
 *
 * Created on: 2024-09-21
 * Author: henry@abx
 */

#ifndef CABX_WEBRTC_CONFIG_H_
#define CABX_WEBRTC_CONFIG_H_

#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#define CONFIG_FILE_PATH					"/opt/abx/config/"
#define ABX_WEBRTC_CONFIG_XML				CONFIG_FILE_PATH"AbxWebRTCConfig.xml"

struct AbxWebRTCSubConfig
{
public:
    bool isEnable;
	int  extInt;
	std::string extStr;

public:
    template< class archive >
    void serialize ( archive &ar, const unsigned ver )
    {
        ar & BOOST_SERIALIZATION_NVP ( isEnable );
        ar & BOOST_SERIALIZATION_NVP ( extInt );
        ar & BOOST_SERIALIZATION_NVP ( extStr );
    }
};

struct AbxWebRTCConfig
{
public:
	bool isEnable;
	int  extInt;
	std::string extStr;
    AbxWebRTCSubConfig abxWebRTCSubConfig;

public:
    template< class archive >
    void serialize ( archive &ar, const unsigned int version )
    {
        ar & BOOST_SERIALIZATION_NVP ( isEnable );
        ar & BOOST_SERIALIZATION_NVP ( extInt );
        ar & BOOST_SERIALIZATION_NVP ( extStr );
        ar & BOOST_SERIALIZATION_NVP ( abxWebRTCSubConfig );
    }
};

class CAbxWebRTCConfig
{
public:
    CAbxWebRTCConfig(void);

public:
    void restoreConfig();
    void loadConfig();
    void saveConfig();
    void generateConfig();

public:
    AbxWebRTCConfig mConfig;
};

#endif /* CABX_WEBRTC_CONFIG_H_ */
