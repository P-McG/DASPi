# Dependencies

## Host (Cross-Compile)

## Target (Shared)

## Aperture Compute Module
The following packages must be installed on the target Raspberry Pi system (e.g., PXE root filesystem or local image).

These provide runtime and development dependencies for `aperturecomputemodule`.

### Base OS

- Raspberry Pi OS Lite (64-bit)

### Install Dependencies

```bash
sudo apt update

sudo apt install -y \
  tmux \
  g++ gcc \
  build-essential libc6-dev \
  libgomp1 \
  libcamera-dev \
  libblas3 liblapack3 \
  libevent-dev libevent-pthreads-2.1-7 libtbb-dev \
  libopencv-core410 libopencv-dev
```
## Compute Module
(specific packages)
