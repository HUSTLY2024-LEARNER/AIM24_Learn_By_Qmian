#!/bin/bash

NET_PORT=eth0

ifconfig_result=$(ifconfig $NET_PORT)
line_count=$(echo $ifconfig_result | grep -c ifconfig_result)

if [[ $line_count != 0 ]]; then
	# 找到网卡
else
	# 没找到网卡
fi
