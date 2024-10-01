cmake_minimum_required(VERSION 3.16.3)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Modules/CangoMake")
include("Cango")

macro (AddGxiAPI)
	if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
		add_library(_gxiapi SHARED IMPORTED)
		set_target_properties(_gxiapi PROPERTIES 
			IMPORTED_LOCATION "/usr/lib/libgxiapi.so"
		)
		include_directories("${PROJECT_SOURCE_DIR}/Thirdparties/gxiapi_linux/include")
		set(gxiapi _gxiapi)
	elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
		add_library(_gxiapi STATIC IMPORTED)
		set_target_properties(_gxiapi PROPERTIES 
			IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/Thirdparties/gxiapi_windows/lib/GxIAPI.lib"
		)
		add_library(_dximageproc STATIC IMPORTED)
		set_target_properties(_dximageproc PROPERTIES 
			IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/Thirdparties/gxiapi_windows/lib/DxImageProc.lib"
		)
		set(gxiapi _gxiapi _dximageproc)
		include_directories("${PROJECT_SOURCE_DIR}/Thirdparties/gxiapi_windows/include")
	else()
		message("Unsupported system for gxiapi")
	endif()
endmacro()

project(LangYa
	VERSION 1.0.2
	DESCRIPTION "Everything by LangYa"
	HOMEPAGE_URL "https://github.com/HUSTLYRM/LangYa"
)

OverrideGlobalCXXStandard(20)
GuardGlobalBuildDirectory()
DefineBuildSystemMacros()
SuppressWindowsWarnings()

find_package(fmt 10.2 CONFIG REQUIRED)
find_package(spdlog 1.14 CONFIG REQUIRED)
find_package(Boost 1.85 REQUIRED COMPONENTS system filesystem json program_options)
find_package(OpenCV 4 REQUIRED)
find_package(Taskflow 3.7 CONFIG REQUIRED)
find_package(OpenVINO CONFIG REQUIRED COMPONENTS Runtime)
find_package(Eigen3 3.4 CONFIG REQUIRED)
find_package(Sophus 1.22 CONFIG REQUIRED)
find_package(Ceres 2.1 CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
#find_package(Doxygen REQUIRED)
#find_package(Sphinx REQUIRED)

AddGxiAPI()

AddModulesInDirectory(
	DIRECTORY "Thirdparties"
	NAMES
		"imgui-docking-1.90.8"
#		"lua-5.4.7"
)

AddModulesInDirectory(
	DIRECTORY "Modules"
	NAMES
		"Cango.CommonUtils"
		"Cango.TaskDesign"
		"Cango.ByteCommunication"
		"Cango.GalaxySDK"
)

AddModulesInDirectory(
	DIRECTORY "Sources"
	NAMES
		"Common" 
		"ArmorDetectors"
		"AutoAim"
#		"ROSHow"
)

#add_subdirectory("Sango")
