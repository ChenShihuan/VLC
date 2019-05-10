#!/bin/bash

CURDIR=`pwd`
echo "Your current directory is $CURDIR. This is where the MVSDK software will be installed..."
A=`whoami`
B=`getconf LONG_BIT`

if [ $A != 'root' ]; then
   echo "You have to be root to run this script"
   echo "Fail !!!"
   exit 1;
fi


cp 88-mvusb.rules /etc/udev/rules.d/

if [ $B == '64' ]; then
	cp 64bit/libMVSDK.so  /lib
	echo "Copy 64bit libMVSDK.so to /lib"
else
	cp 32bit/libMVSDK.so  /lib
	echo "Copy 32bit libMVSDK.so to /lib"
fi
echo "Successful"
echo "Please  restart system  now!!!"
