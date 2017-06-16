#!/bin/bash

if [ $1 = "" ]; then
	echo "Usage: ./catStream.sh [delay in milliseconds]"
	exit 0
fi

echo "Listening on port 5000"

nc -l 5000 | delay $1 | mplayer -fps 200 -demuxer h264es -
