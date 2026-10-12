To run the cameras directly from the aperturecomputemoduleXXX to computemoduleXXX

On aperturecomputemoduleXXX, run:
```
rpicam-vid -t 0 -n \
  --width 1280 --height 720 \
  --framerate 30 \
  --codec yuv420 \
  -o - | \
ffmpeg -f rawvideo \
  -pixel_format yuv420p \
  -video_size 1280x720 \
  -framerate 30 \
  -i pipe:0 \
  -c:v libx264 \
  -preset ultrafast \
  -tune zerolatency \
  -g 30 -bf 0 \
  -f mpegts \
  "udp://10.0.2.1:8888?pkt_size=1316"
```

This pipeline works as follows:
IMX296 global shutter camera
Raspberry Pi 5 — 1280 × 720 at 30 FPS

rpicam-vid
Uncompressed YUV420

FFmpeg — libx264
Software H.264 encoding

UDP → 10.0.2.1:8888
MPEG-TS video stream to Ubuntu laptop

On the Ubuntu laptop, start the receiver first:
```
ffplay -fflags nobuffer -flags low_delay \
  -framedrop -f mpegts \
  "udp://@:8888"
```
