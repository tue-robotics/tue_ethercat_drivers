cmake_minimum_required(VERSION 2.8.3)
project(tue_ethercat_drivers)

find_package(catkin REQUIRED 
	soem_master
 )

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS} )
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(soem REQUIRED)
include_directories(${soem_INCLUDE_DIRS})

orocos_use_package(soem_master)

#ros_generate_rtt_typekit(tue_ethercat_drivers)

orocos_plugin(TUeES030 src/soem_beckhoff_drivers.cpp src/TUeEthercatSlave030.cpp)
target_link_libraries(TUeES030 ${soem_LIBRARIES})
# does not build:
#orocos_plugin(DummyDrivers src/soem_beckhoff_drivers.cpp src/DummyDrivers.cpp)
#target_link_libraries(DummyDrivers ${soem_LIBRARIES})

orocos_generate_package() 
