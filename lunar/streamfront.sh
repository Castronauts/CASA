gst-launch-1.0 -v tcpclientsrc host=192.168.1.3 port=5000 ! gdpdepay ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
