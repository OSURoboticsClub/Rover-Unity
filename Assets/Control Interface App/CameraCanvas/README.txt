gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480 ! jpegparse ! udpsink host=127.0.0.1 port=8080

This command above is a general example of taking your laptop webcam and exposing a jpeg stream to port 8080 on your localhost

On the unity side, there is a script in this directory that attatches to object you want the camera feed to show up on. 

The 3 already in use on the Unity side are - 8080,8081,8082.

This pipeline can be expanded to the rover by having a udpsrc gstreamer intake component, with a similar udpsink output to stream directly to Unity.
