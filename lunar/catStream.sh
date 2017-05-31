#!/bin/bash

nc -l 5000 | delay 2300 | mplayer -fps 200 -demuxer h264es -
