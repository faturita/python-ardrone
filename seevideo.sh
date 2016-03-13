# Use me to access the H.264 stream from ARDRone.

ffplay -framedrop -infbuf -f h264 -i http://192.168.1.1:5555 -framerate 60
