/*
 * CAbx.h
 *
 * Created on: 2024-07-24
 * Author: henry@abx
 */

#ifndef CABXCONFIG_H_
#define CABXCONFIG_H_

#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#define CONFIG_FILE_PATH					"/opt/abx/config/"
#define ABX_CONFIG_XML				CONFIG_FILE_PATH"AbxConfig.xml"

struct AbxSubConfig
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

struct AbxConfig
{
public:
	bool isEnable;
	int  extInt;
	std::string extStr;
    AbxSubConfig abxSubConfig;

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

class CAbxConfig
{
public:
    CAbxConfig(void);

public:
    void restoreConfig();
    void loadConfig();
    void saveConfig();
    void generateConfig();

public:
    AbxConfig mConfig;
};

#endif /* CABXCONFIG_H_ */
