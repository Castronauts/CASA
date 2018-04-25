height=0
width=0
framerate=0

if [ "$1" = "" ]
then
    echo "Usage: ./video.sh width(in px) height(in px) framerate"
    exit 0
fi 

width="$1" 

if [ "$2" = "" ]
then
    height=$((($width*3)/4))
else
    height="$2" 
fi

if [ "$3" = "" ]
then
    framerate=60
else
    framerate="$3"
fi 


echo "$width" 
echo "$height"
echo "$framerate"

raspivid -t 0 -w "$width" -h "$height" -fps "$framerate" -b 200000000 --nopreview -o - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! gdppay ! udpsink host = 192.168.1.1 port=5000