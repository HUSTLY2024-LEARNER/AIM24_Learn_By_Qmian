#!/bin/bash

project_name=$1
project_name_lower=$(echo ${project_name} | tr '[:upper:]' '[:lower:]')

mkdir ./${project_name}
if [ $? -ne 0 ]; then
	echo "创建${project_name}文件夹失败"
	exit 1
fi

mkdir ./${project_name}/Headers
mkdir ./${project_name}/Headers/LangYa
mkdir ./${project_name}/Headers/LangYa/${project_name}
TEMPLATE_HPP_CONTENT="#pragma once"
echo $TEMPLATE_HPP_CONTENT > ./${project_name}/Headers/LangYa/${project_name}.hpp

mkdir ./${project_name}/Sources
TEMPLATE_CPP_CONTENT="#include <LangYa/${project_name}.hpp>"
echo $TEMPLATE_CPP_CONTENT > ./${project_name}/Sources/${project_name}.cpp

mkdir ./${project_name}/Testers

touch ./${project_name}/CMakeLists.txt
cat << CMAKELISTS_TXT_EOF > ./${project_name}/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(LangYa.${project_name}
	VERSION 1.0.0.0
	DESCRIPTION ""
	HOMEPAGE_URL ""
	LANGUAGES CXX
)

RequireLibrary(LangYa::CodeHelper)

set(${project_name_lower}_name ${project_name})
set(${project_name_lower}_namespace LangYa)
set(${project_name_lower}_cxx_standard 20)
set(${project_name_lower}_links
	PUBLIC LangYa::CodeHelper
)

AddLibraryModule(${project_name_lower})

CMAKELISTS_TXT_EOF
