cmake_minimum_required(VERSION 3.0.2)

project(ROSHow)

list(APPEND CMAKE_PREFIX_PATH
	/opt/ros/noetic/share/catkin/cmake
	/opt/ros/noetic/share/roscpp/cmake
)

find_package(catkin CONFIG REQUIRED)

macro(SearchROSLib libName)
	add_library(_noetic_${libName} SHARED IMPORTED)
	set_target_properties(_noetic_${libName} PROPERTIES
		IMPORTED_LOCATION /opt/ros/noetic/lib/lib${libName}.so
	)
	add_library(noetic::${libName} ALIAS _noetic_${libName})
endmacro()

SearchROSLib(cpp_common)
SearchROSLib(roscpp)
SearchROSLib(rosbag)
SearchROSLib(roslib)
SearchROSLib(roslz4)
SearchROSLib(rostime)
SearchROSLib(rosconsole)
SearchROSLib(roscpp_serialization)

add_executable(LangYa_ROSHow_main Sources/main.cpp)
target_link_directories(LangYa_ROSHow_main
	PUBLIC /opt/ros/noetic/lib
)
target_link_libraries(LangYa_ROSHow_main 
	PUBLIC noetic::cpp_common
	PUBLIC noetic::roscpp
	PUBLIC noetic::rosbag
	PUBLIC noetic::roslz4
	PUBLIC noetic::roslib
	PUBLIC noetic::rostime
	PUBLIC noetic::rosconsole
	PUBLIC noetic::roscpp_serialization
	PUBLIC LangYa::Common
)
target_include_directories(LangYa_ROSHow_main BEFORE 
	PUBLIC /opt/ros/noetic/include
)
