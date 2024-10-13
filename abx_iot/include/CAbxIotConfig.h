/*
 * CAbxIotConfig.h
 *
 * Created on: 2024-10-13
 * Author: henry@abx
 */

#ifndef CABX_IOT_CONFIG_H_
#define CABX_IOT_CONFIG_H_

#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#define CONFIG_FILE_PATH				"/opt/abx/config/"
#define ABX_IOT_CONFIG_XML				CONFIG_FILE_PATH"AbxIoTConfig.xml"

struct AbxIotSubConfig
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

struct AbxIotConfig
{
public:
	bool isEnable;
	int  extInt;
	std::string extStr;
    AbxIotSubConfig abxSubConfig;

public:
    template< class archive >
    void serialize ( archive &ar, const unsigned int version )
    {
        ar & BOOST_SERIALIZATION_NVP ( isEnable );
        ar & BOOST_SERIALIZATION_NVP ( extInt );
        ar & BOOST_SERIALIZATION_NVP ( extStr );
        ar & BOOST_SERIALIZATION_NVP ( abxSubConfig );
    }
};

class CAbxIotConfig
{
public:
    CAbxIotConfig(void);

public:
    void restoreConfig();
    void loadConfig();
    void saveConfig();
    void generateConfig();

public:
    AbxIotConfig mConfig;
};

#endif /* CABX_IOT_CONFIG_H_ */
