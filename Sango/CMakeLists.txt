cmake_minimum_required(VERSION 3.28)

project(Sango
	VERSION 0.1.0.0
	DESCRIPTION "Everything by Sango"
	HOMEPAGE_URL ""
)

find_package(SDL2 CONFIG REQUIRED)

AddModulesInDirectory(
	DIRECTORY "Sources"
	NAMES
#		"SangoConsole"
#		"SangoLua"
#		"Assistant"
		"GraphicUserInterface"
#		"SDLLearning"
)

