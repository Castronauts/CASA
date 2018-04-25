# Raspberry Pi Video Streaming

Some of these files may need to be modified to fit your needs later. In particular the raspberry pi and client computer assume particular IP addresses and port numbers for sending the video stream. When setting up a new video stream these parameters will most likely need to be changed. With that being said here is the typical flow of the video stream.

raspi cam video capture &rightarrow; netcat data transfer &rightarrow; netcat data reception &rightarrow; delay pipe &rightarrow; mplayer display on client

More detail of each side below

### Raspi (server) Side

- The built in command line interface `raspivid` is used to capture a video stream for the raspi. Paramters can be specified for the type of video to be captured. Make sure the video stream is outputted to stdout with the `-o` option.

- Pipe the video stream into `nc` (netcat) which will send the video stream to the specified IP and port number

### Client side

- Receive the video stream by listening on the correct port number using `nc` (netcat) and pipe the output to the __*[delay]*__ module.

- The __*[delay]*__ module is a simple command line interface that delays its standard input to standard output by the time you specify. This is handy for introducing a custom amount of latency. Pipe the output of __*[delay]*__ into `mplayer`.

- `mplayer` is a multimedia player used in this case to display the raspberry pi video stream. `raspivid` encodes the video using h.264 so the `-demuxer h264es` option must be used to decode the video stream.

[delay]:https://github.com/rom1v/delay
