cmake_minimum_required(VERSION 2.8.3)
project(abx)

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
	src/CAbx.cpp
	src/CAbxConfig.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pthread
  boost_system
  boost_filesystem
  boost_serialization
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

