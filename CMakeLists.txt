cmake_minimum_required(VERSION 2.8.3)
project(tue_ethercat_drivers)

find_package(catkin REQUIRED 
  soem_master
  roscpp
  soem_beckhoff_drivers
)

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS} )
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(soem REQUIRED)
include_directories(${soem_INCLUDE_DIRS})

file(GLOB_RECURSE HEADER_FILES include/*.hpp)

orocos_use_package(soem_master)

# TUe EtherCAT print drivers
orocos_plugin(TUeES020 src/soem_beckhoff_drivers.cpp src/TUeES020.cpp ${HEADER_FILES})
target_link_libraries(TUeES020 ${soem_LIBRARIES})
orocos_plugin(TUeES030 src/soem_beckhoff_drivers.cpp src/TUeES030.cpp ${HEADER_FILES})
target_link_libraries(TUeES030 ${soem_LIBRARIES})

# Dummy Drivers
orocos_plugin(TUeEC010 src/soem_beckhoff_drivers.cpp src/TUeEC010_dummy.cpp ${HEADER_FILES})
target_link_libraries(TUeEC010 ${soem_LIBRARIES})
orocos_plugin(EK1100 src/soem_beckhoff_drivers.cpp src/EK1100_dummy.cpp ${HEADER_FILES})
target_link_libraries(EK1100 ${soem_LIBRARIES})

# Wait for message generation
add_dependencies(TUeES020 ${catkin_EXPORTED_TARGETS})
add_dependencies(TUeES030 ${catkin_EXPORTED_TARGETS})
add_dependencies(TUeEC010 ${catkin_EXPORTED_TARGETS})
add_dependencies(EK1100   ${catkin_EXPORTED_TARGETS})

orocos_generate_package() 
