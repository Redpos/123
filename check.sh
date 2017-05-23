#!/bin/bash

while true
do
	if pgrep -x "ipconvif" > /dev/null
	then
		#echo "Running"
		sleep 1
	else
		echo "Starting up"
		cmd="./ipconvif -cIp 192.168.11.41 -cUsr admin -cPwd Supervisor"
		nohup $cmd &
		sleep 5
	fi
done
