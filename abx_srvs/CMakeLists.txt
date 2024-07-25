cmake_minimum_required(VERSION 2.8.3)
project(abx_srvs)

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  std_msgs
)

add_service_files(
	FILES

	abx_auth.srv
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(CATKIN_DEPENDS 
	message_runtime 
	std_msgs
)

install(FILES CHANGELOG.rst
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
