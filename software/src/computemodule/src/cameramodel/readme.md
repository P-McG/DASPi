# prerequisites
## openmp
## eigen3
sudo apt install libeigen3-dev
## opencv
sudo apt update && sudo apt upgrade -y
sudo apt install libopencv-dev -y
# Build commands
From that directory:
```
meson setup build
meson compile -C build
```
# Run it with:
```
./build/sphere_stitcher_demo
```
## View bayer
```
ffplay -f rawvideo -pixel_format bayer_rggb16le -video_size 1456x1088 ~/bayer_dumps/output-10.0.2.3_0.bayer
```
