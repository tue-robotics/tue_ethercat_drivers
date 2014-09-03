cmake_minimum_required(VERSION 2.8.3)
project(tue_ethercat_drivers)

find_package(catkin REQUIRED 
	soem_master
	soem_beckhoff_drivers
 )

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS} )
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

find_package(soem REQUIRED)
include_directories(${soem_INCLUDE_DIRS})

orocos_use_package(soem_master)

#ros_generate_rtt_typekit(soem_beckhoff_drivers)

#orocos_plugin(TUeES030 ${soem_beckhoff_drivers_PACKAGE_PATH}/src/soem_beckhoff_drivers.cpp TUeEthercatSlave030.cpp)
#target_link_libraries(TUeES030 ${soem_LIBRARIES})
#add_dependencies(TUeES030 ${PROJECT_NAME}_generate_messages_cpp)

orocos_generate_package(
  INCLUDE_DIRS include
) 
