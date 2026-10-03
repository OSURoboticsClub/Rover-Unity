gst-launch-1.0 tcpclientsrc port=42073 host=192.168.1.11      ! \
    "application/x-rtp-stream,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H265" ! \
    rtpstreamdepay ! \
    rtpjitterbuffer latency=200 ! \
    rtpulpfecdec ! \
    rtph265depay ! \
    h265parse ! \
    queue max-size-buffers=30 max-size-time=0 max-size-bytes=0 leaky=downstream ! \
    avdec_h265 ! \
    videoconvert ! \
    videorate ! \
    video/x-raw,framerate=30/1 ! \
    autovideosink sync=false &
gst-launch-1.0 tcpclientsrc port=42070 host=192.168.1.11      ! \
    "application/x-rtp-stream,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H265" ! \
    rtpstreamdepay ! \
    rtpjitterbuffer latency=200 ! \
    rtpulpfecdec ! \
    rtph265depay ! \
    h265parse ! \
    queue max-size-buffers=30 max-size-time=0 max-size-bytes=0 leaky=downstream ! \
    avdec_h265 ! \
    videoconvert ! \
    videorate ! \
    video/x-raw,framerate=25/1 ! \
    autovideosink sync=false &
gst-launch-1.0 tcpclientsrc port=42071 host=192.168.1.11      ! \
    "application/x-rtp-stream,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H265" ! \
    rtpstreamdepay ! \
    rtpjitterbuffer latency=200 ! \
    rtpulpfecdec ! \
    rtph265depay ! \
    h265parse ! \
    queue max-size-buffers=30 max-size-time=0 max-size-bytes=0 leaky=downstream ! \
    avdec_h265 ! \
    videoconvert ! \
    videorate ! \
    video/x-raw,framerate=25/1 ! \
    autovideosink sync=false &
gst-launch-1.0 tcpclientsrc port=42068 host=192.168.1.11      ! \
    "application/x-rtp-stream,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H265" ! \
    rtpstreamdepay ! \
    rtpjitterbuffer latency=200 ! \
    rtpulpfecdec ! \
    rtph265depay ! \
    h265parse ! \
    queue max-size-buffers=30 max-size-time=0 max-size-bytes=0 leaky=downstream ! \
    avdec_h265 ! \
    videoconvert ! \
    videorate ! \
    video/x-raw,framerate=30/1 ! \
    autovideosink sync=false &
wait

