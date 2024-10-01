#!/bin/bash

CFG_USER=hustlyrm
CFG_FILE_PATH=/media/$CFG_USER/*/.shutdown

var_found_file=false
echo "[#] DetectingShutdown when file exists"

while [ "$var_found_file" = false ]; do
	ls_result=$(ls $CFG_FILE_PATH)
	if [ "$ls_result" == "" ]; then
		sleep 2
	else
		var_found_file=true
	fi
done

echo "[#] executing shutdown"
shutdown -h now
