#!/bin/bash

nc -l 5000 | delay 2300 | mplayer -fps 200 -delay 2.5 -demuxer h264es -
