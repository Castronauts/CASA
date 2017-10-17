#!/bin/bash

if [ "$1" = "" ]; then
	echo "Usage: ./topcatStream.sh [delay in milliseconds]"
	exit 0
fi

echo "Listening on port 5001"

nc -l 5001 | delay $1 | mplayer -fps 200 -demuxer h264es -
