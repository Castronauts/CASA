#!/bin/bash

nc -l 5001 | delay 2300 | mplayer -fps 200 -demuxer h264es -
