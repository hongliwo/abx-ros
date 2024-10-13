cmake_minimum_required(VERSION 3.1)
project(abx_iot)

add_compile_options(-std=c++11)


set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(
    catkin
    REQUIRED
    COMPONENTS
    roscpp
	std_msgs
	std_srvs
    abx_msgs
    abx_srvs
)

catkin_package(
    CATKIN_DEPENDS
    abx_msgs
	abx_srvs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

link_directories(
    ${catkin_LIB_DIRS}
    lib
)

add_executable(${PROJECT_NAME}
    src/main.cpp
	src/CAbxIot.cpp
	src/CAbxIotConfig.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wno-long-long -pedantic -Werror)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pthread
  boost_system
  boost_filesystem
  boost_serialization
  -Wl,--whole-archive
  aws-c-auth
  aws-c-cal
  aws-c-common
  aws-c-compression
  aws-c-event-stream
  aws-checksums
  aws-c-http
  aws-c-io
  aws-c-iot
  aws-c-mqtt
  aws-crt-cpp
  aws-c-s3
  aws-c-sdkutils
  crypto
  Discovery-cpp
  EventstreamRpc-cpp
  GreengrassIpc-cpp
  IotDeviceCommon-cpp
  IotDeviceDefender-cpp
  IotIdentity-cpp
  IotJobs-cpp
  IotSecureTunneling-cpp
  IotShadow-cpp
  s2n
  dl
  -Wl,--no-whole-archive
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES CHANGELOG.rst
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#catkin_metapackage()

