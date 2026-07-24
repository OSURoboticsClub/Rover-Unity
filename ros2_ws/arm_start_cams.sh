# IP needs to be 192.168.1.8
gst-launch-1.0 udpsrc port=42072 caps="application/x-rtp, media=video, encoding-name=H265, payload=96" ! \
    rtpjitterbuffer latency=200 ! \
    rtpulpfecdec ! \
    rtph265depay ! \
    h265parse ! \
    queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 ! \
    avdec_h265 ! \
    videoconvert ! \
    videorate ! \
    video/x-raw,framerate=30/1 ! \
    autovideosink sync=false &

gst-launch-1.0 udpsrc port=42074 caps="application/x-rtp, media=video, encoding-name=H265, payload=96" ! \
    rtpjitterbuffer latency=200 ! \
    rtpulpfecdec ! \
    rtph265depay ! \
    h265parse ! \
    queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 ! \
    avdec_h265 ! \
    videoconvert ! \
    videorate ! \
    video/x-raw,framerate=30/1 ! \
    autovideosink sync=false &

gst-launch-1.0 udpsrc port=42068 caps="application/x-rtp, media=video, encoding-name=H265, payload=96" ! \
    rtpjitterbuffer latency=200 ! \
    rtpulpfecdec ! \
    rtph265depay ! \
    h265parse ! \
    queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 ! \
    avdec_h265 ! \
    videoconvert ! \
    videorate ! \
    video/x-raw,framerate=30/1 ! \
    autovideosink sync=false &

wait
