# DASPi
## What it is
Distributed Aperture System (DAS) using the Raspberry Pi.
DASPi consists of n-cameras mounted around the physical object in such a way as to provide unobstructed spherical (4π steradian) coverage.

**Very early work in progress**

## Camera Pipeline

The DASPi camera pipeline captures raw Bayer frames on the aperturecomputemodule, applies per-pixel processing in raw space, sends compact masked regions over UDP, then reconstructs and demosaics them on the computemodule.

### 1. Camera capture on `aperturecomputemodule`

The aperturecomputemodule uses `libcamera` with a raw stream role.

For Raspberry Pi 5 / PiSP, the raw stream must be requested in **unpacked** mode. In practice this means validating to:

- pixel format: `SBGGR16`
- resolution: `1456 x 1088`
- stride: `2944`

This is important because PiSP can otherwise return compressed raw formats such as `BGGR_PISP_COMP1`, which cannot be treated as a linear `uint16_t` image buffer.

Although the sensor is effectively 10-bit raw, in unpacked mode the samples are delivered in a 16-bit container. On Pi 5 this behaves like the 10-bit values are left-shifted into the 16-bit range.

### 2. Frame buffer readout

Each completed `libcamera::FrameBuffer` is mapped and copied row-by-row into a contiguous `std::vector<uint16_t>` called `activeFrame`.

This copy step removes per-row stride padding from the camera buffer, so downstream code works with a dense `width * height` buffer instead of a strided plane.

Conceptually:

- source: strided raw plane from `libcamera`
- destination: contiguous `activeFrame`

### 3. Raw-space processing on `aperturecomputemodule`

Once `activeFrame` is available, the pipeline performs processing directly on the Bayer mosaic:

- facet masking / region extraction
- white balance on mosaic samples
- packing masked regions into a `ShapeFunctionDataPacket`

White balance is applied before demosaic, using Bayer parity from the global sensor coordinates.

### 4. UDP transmission

The aperturecomputemodule does not send the full raw frame.

Instead, it sends compact masked regions over UDP:

- frame metadata
- gain metadata
- per-region valid sizes
- contiguous `uint16_t` payload data

This reduces bandwidth compared to transmitting the full frame.

### 5. Frame reconstruction on `computemodule`

The computemodule receives UDP chunks, reorders frames, reconstructs the region payloads, and unpacks them back into unmasked image buffers.

At this stage the buffers are already contiguous `std::vector<uint16_t>` images. Camera-side stride alignment and padding are no longer present.

One important detail is that the Pi 5 unpacked raw stream already arrives in a 16-bit-scaled form, so the computemodule must **not** apply an additional `<< 6` brightness shift.

### 6. Demosaic on `computemodule`

After reconstruction, the raw Bayer frame is converted to color using OpenCV.

Example:

```cpp
cv::cvtColor(raw16, bgr16, cv::COLOR_BayerRG2BGR);
```

## Current Hardware
### Compute Module
Raspberry Pi 5 
Raspberry Pi AI Kit
Dedicated All-In-One Aluminum Cooler for Raspberry Pi 5, PWM
### Aperture Compute Module
Raspberry Pi Zero 2 W
Raspberry Pi Global Shutter Camera: imx296
Dedicated Aluminum Heatsink for Raspberry Pi Zero 2 W
Raspberry Pi Camera Cable Standard - Mini - 200mm V2
### Ethernet Switch
TP-Link TL-SG105S-M2 | 5-Port 2.5G Multi-Gigabit Unmanaged Network Switch
## Future Hardware
Pi 5 + NVMe HAT = ideal PXE server
final setup
Server Pi
Pi 5 (server)
- NVMe storage
- static IP (10.0.2.2)
- runs:
   dnsmasq
   nfs-kernel-server
   auto-provision service
Clients
Pi 5 nodes
- no SD card
- netboot only
- each gets:
   /srv/nfs/node-* 
##Software
### Repo layout
```.
├── LICENSE
├── README.md
├── software
│   ├── build
│   ├── dist
│   │   ├── aperturecomputemodule
│   │   │   └── aperturecomputemodule
│   │   └── computemodule
│   │       └── computemodule
│   ├── docs
│   ├── scripts
│   │   ├── camera_dashboard.sh
│   │   ├── common_deploy.sh
│   │   ├── distribute_and_run_aperturecomputemodule_code.sh
│   │   ├── distribute_and_run_computemodule_code.sh
│   │   ├── fetch_files.sh
│   │   ├── monitor_aperture_nodes.sh
│   │   └── update_sysroot.sh
│   └── src
│       ├── aperturecomputemodule
│       │   ├── cross-aperturecomputemodule.txt
│       │   ├── include
│       │   ├── meson.build
│       │   ├── pch
│       │   ├── src
│       │   └── TODO.md
│       ├── common
│       │   ├── include
│       │   ├── logger
│       │   ├── meson.build
│       │   ├── scopedtimer
│       │   ├── shapefunction
│       │   └── udp
│       ├── computemodule
│       │   ├── cross-computemodule.txt
│       │   ├── include
│       │   ├── meson.build
│       │   ├── src
│       │   └── TODO.md
│       ├── meson.build
│       └── meson_options.txt
├── structure.txt
└── TODO.md

250 directories, 132 files
```

### Build prerequisites for Compute-Module

-Raspberry Pi OS Lite (64-bit),

sudo apt install dnsmasq

-threads
-tbb
-opencv,
-eigen,
-openmp,
-blas,
-lapack,
#### Host file On host
Place in host file:

```
[ip_address_here] computemodule000
[ip_address_here] aperturecomputemodule000
.                      .
.                      .
.                      .
[ip_address_here] aperturecomputemoduleNNN
```

sudo apt update
sudo apt install -y tmux

### Build prerequisites for Aperture-Compute-Module
-Raspberry Pi OS Lite (64-bit),
```
sudo apt update
sudo apt install -y tmux
sudo apt install -y g++ gcc
sudo apt install -y build-essential libc6-dev
sudo apt install -y libgomp1
sudo apt install -y libcamera-dev
sudo apt install -y libblas3 liblapack3
sudo apt install -y libevent-dev libevent-pthreads-2.1-7 libtbb-dev
sudo apt install -y libopencv-core410 libopencv-dev 
```
### Build golden image
Aperture-Compute-Module(Pi):
  install packages (eg. libcamera-dev)
Development Machine:
→ build image
→ refresh sysroot
→ build on dev machine using sysroot
→ deploy binary
```
~/DASPi/images/aperturecomputemodule/builds/build-image.sh
```
### Generate sysroots
```
./refresh-sysroot.sh
./verify-sysroot.sh
```
### Image to SD card.
```
sudo umount /dev/mmcblk0*
sudo dd if=../golden/aperturecomputemodule.img of=/dev/mmcblk0 bs=4M oflag=direct conv=fsync status=progress
```
#### Native prerequisites
```
sudo apt install \
  build-essential meson ninja-build pkg-config \
  libopencv-dev libeigen3-dev libtbb-dev \
  libblas-dev liblapack-dev
```

#### Host file On host
Place in host file:
```
[ip_address_here] computemodule000
[ip_address_here] aperturecomputemodule000
.                      .
.                      .
.                      .
[ip_address_here] aperturecomputemoduleNNN
```
#### network configuration
```
gateway = 10.0.2.1
client  = 10.0.2.2
server0 = 10.0.2.3
server1 = 10.0.2.4
server2 = 10.0.2.5
...
```

#### Coss-Compiling

##### prerequisites
sudo apt update
sudo apt install kpartx

#### Cross-Compiling Sources for the Compute-Module and Aperture-Compute-Module
Set up the cross compiling sources for the aperturecomputemodule000. 
This needs to be performed every major change, or update, to the aperturecomputemodules. 
Only one of the aperturecomputemodules is needed to be synced if all of the aperturecomputemodules are the same. 

```
~/DASPi/software/scripts/update_sysroot.sh
```

### Cross-Compiling aperturecomputemodule
```
rm -rf ~/DASPi/software/build/aperturecomputemodule

meson setup ~/DASPi/software/build/aperturecomputemodule \
  ~/DASPi/software/src \
  --cross-file ~/DASPi/software/src/aperturecomputemodule/cross-aperturecomputemodule.txt \
  -Dbuild_aperturecomputemodule=true \
  -Dbuild_computemodule=false

meson compile -C ~/DASPi/software/build/aperturecomputemodule
```
To send just the binary run:
```scp ~/DASPi/software/build/aperturecomputemodule/aperturecomputemodule/aperturecomputemodule     daspi@10.0.2.10:~```
Or,
```rsync -avz aperturecomputemodule daspi@10.0.2.10:~```
Then on the Pi:
```
sudo mv ~/aperturecomputemodule /opt/daspi/
sudo chmod +x /opt/daspi/aperturecomputemodule
```


### Manually running aperturecomputemodule service
```sudo systemctl start aperturecomputemodule```
Check status:
```systemctl status aperturecomputemodule```
Follow logs live:
```journalctl -u aperturecomputemodule -f```
Running the underlining binary
```sudo /opt/daspi/aperturecomputemodule```

### Cross-compiling computemodule 
```
rm -rf ~/DASPi/software/build/computemodule

meson setup ~/DASPi/software/build/computemodule \
  ~/DASPi/software/src \
  --cross-file ~/DASPi/software/src/computemodule/cross-computemodule.txt \
  -Dbuild_aperturecomputemodule=false \
  -Dbuild_computemodule=true

meson compile -C ~/DASPi/software/build/computemodule
```
### Deployment
To copy output from the computemodule
```
~/DASPi/software/scripts/fetch_files.sh
```
### Native compiling computemodule

1. Enable forwarding
```sudo sysctl -w net.ipv4.ip_forward=1```
2. Add NAT (this is the critical step)
```sudo iptables -t nat -A POSTROUTING -o wlp0s20f3 -j MASQUERADE```
3. Allow forwarding
```sudo iptables -A FORWARD -i wlp0s20f3 -o enp0s31f6 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i enp0s31f6 -o wlp0s20f3 -j ACCEPT```

#### Build and Compile
```
rm -rf ~/DASPi/software/build/computemodule-native

meson setup ~/DASPi/software/build/computemodule-native \
  ~/DASPi/software/src \
  -Dbuild_computemodule=true \
  -Dbuild_aperturecomputemodule=false
  
meson compile -C ~/DASPi/software/build/computemodule-native

~/DASPi/software/build/computemodule-native/computemodule/computemodule --nApertureComputeModules=2 --usbBaseIp=10.0.2.1 --port=5000
```
###
```
scp ~/DASPi/software/build/aperturecomputemodule/aperturecomputemodule/aperturecomputemodule     daspi@10.0.2.10:/tmp/
```
### Viewing captured output
```
ffplay -f rawvideo -pixel_format bayer_rggb16le -video_size  \
1456x1088 output-10.0.2.3_0.bayer
```

### Updating aperturecomputemodule image FROM Pi SD card
```
sudo umount /dev/mmcblk0p1 2>/dev/null || true
sudo umount /dev/mmcblk0p2 2>/dev/null || true
sync

sudo dd if=/dev/mmcblk0 of=~/DASPi/images/aperturecomputemodule/golden/aperturecomputemodule.img bs=4M status=progress conv=fsync
sync
```
### Updating sysroot from image.
```
~/DASPi/images/aperturecomputemodule/builds/refresh-sysroot.sh
~/DASPi/images/aperturecomputemodule/builds/verify-sysroot.sh
```
### Network boot by synServing disk image to Aperture Compute Module(Pi)

Boot ROM → DHCP → TFTP → boot files → NFS/HTTP root

[ApertureComputeModule]
        │
        ▼
   DHCP request
        │
        ▼
   PXE Server ()
   ├── DHCP (dnsmasq)
   ├── TFTP (boot files)
   ├── Initramfs
   └── NFS (root filesystem)
   
#### Zero-touch provisioning for new Raspberry Pi 5 nodes.

##### Nodes first boot “EEPROM-primed”:
For Pi 5 You must prime the EEPROM using an SD card one time.

**Step 1** — insert SD card and boot

Use your working OS image.
``` 
arp -a
ssh daspi@[IP ADDRESS]
```
**Step 2** — configure EEPROM

Run:
```sudo -E rpi-eeprom-config --edit```

Set:
```
BOOT_ORDER=0xf21
BOOT_UART=1
NET_INSTALL_AT_POWER_ON=1
```
**Step 3** — ensure EEPROM is updated
```sudo rpi-eeprom-update```

If update available:
```sudo rpi-eeprom-update -a
sudo reboot```
**Step 4** — power off and remove SD
```sudo poweroff```
Remove SD card.
**Step 5** — power on (no SD)
Now you should see:
Ethernet LEDs turn on
DHCP traffic in on server:
'''journalctl -u dnsmasq -f'''


new Pi → auto-detected
rootfs cloned
TFTP folder created
boots fully

### PXE/NFS server for the entire cluster

Migration plan (from your current setup)
**Step 1** — flash new SD card
Use:
Raspberry Pi Imager
Flash:
Raspberry Pi OS (Lite is fine)
**Step 2** — boot new server Pi
Set:
hostname (optional)
SSH enabled
network working
**Step 3** — prepare server Pi

Install:
```sudo apt install dnsmasq nfs-kernel-server```
**Step 4** — recreate directories
```sudo mkdir -p /srv/nfs /srv/tftp```
**Step 5** — copy your working setup
```
sudo rsync -a \
  --rsync-path="sudo rsync" \
  /srv/tftp/ \
  daspi@<pxeserver_ip>:/srv/tftp/
sudo rsync -a \
  --rsync-path="sudo rsync" \
  /srv/nfs/ \
  daspi@<pxeserver_ip>:/srv/nfs/
```  
**Step 6** — copy configs
scp /etc/dnsmasq.conf daspi@<pxeserver_ip>:/tmp/
scp /etc/exports daspi@<pxeserver_ip>:/tmp/

Then on new Pi:

sudo mv /tmp/dnsmasq.conf /etc/dnsmasq.conf
sudo mv /tmp/exports /etc/exports
```/etc/dnsmasq.conf
/etc/exports
/usr/local/bin/pi-auto-provision.sh```
**Step 7** — set static IP

Example:
10.0.2.2
**Step 5** — plug nodes into Pi network
⚡ Optional upgrade (very nice)

Use two interfaces:

eth0 → cluster network (PXE)
wlan0 → internet

### GIT Commits Notes:
Sync workflow 
Before working:
git pull
After changes:
git add .
git commit -m "message"
git push

## Current status / roadmap

### Roadmap
-[ ] represent each overlap/non-overlap region as an image-space mask per camera
-[ ] create one common world frame
-[ ] use calibrated unprojection for each wide-angle image
-[ ] generate an equirectangular sphere map by inverse mapping
-[ ] use non-overlap masks for hard ownership
-[ ] use overlap masks for feathered blending
