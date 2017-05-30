#!/bin/bash

nc -l 5000 | mplayer -fps 200 -delay 2.0 -demuxer h264es -