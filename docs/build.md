# DASPi - Build
The following is for building the two components. The Aperture-Compute-Module and the Compute-Module. The Aperture-Compute-Module the collects and sends the image data to the Compute-Module. The Compute-module assembles the frames into a spherical image and deploys it to the final viewer.

## Cross-Compilation
This project is developed on a Linux host and cross-compiled for the Raspberry Pi target using an **AArch64 (arm64)** toolchain.

## Overview
The `aperturecomputemodule` target is built on the host machine but linked against libraries from the Raspberry Pi target filesystem using a **sysroot**. This allows development and compilation to happen on the laptop or workstation while producing binaries that run on the Pi.

Cross-compilation is used for:

- faster iteration on the host machine
- avoiding long native builds on the Pi
- keeping the target build aligned with the deployed Raspberry Pi image

## Toolchain
The build uses an AArch64 cross-compiler such as:

```bash
aarch64-linux-gnu-gcc
aarch64-linux-gnu-g++
```
Typical packages on Ubuntu:
```bash
sudo apt update
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu pkg-config meson ninja-build
```

## Sysroot
A sysroot is a copy of the target Raspberry Pi root filesystem used during cross-compilation. It provides:

-target headers
-target shared libraries
-pkg-config metadata
-libcamera / OpenCV / other runtime dependencies

The sysroot should match the software installed on the target image as closely as possible.

Example layout:
```
~/DASPi/images/aperturecomputemodule/sysroot/rootfs/
├── usr/include
├── usr/lib/aarch64-linux-gnu
├── lib/aarch64-linux-gnu
└── usr/lib/pkgconfig
```
## Getting the Sysroot
A practical way to create or refresh the sysroot is to copy it from the Raspberry Pi target root filesystem.

Example using rsync from a running Pi:
```bash
rsync -aHAX --numeric-ids --info=progress2 \
  daspi@aperturecomputemodule000:/ \
  ~/DASPi/images/aperturecomputemodule/sysroot/rootfs \
  --exclude=/dev \
  --exclude=/proc \
  --exclude=/sys \
  --exclude=/tmp \
  --exclude=/run \
  --exclude=/mnt \
  --exclude=/media \
  --exclude=/lost+found
```

If the deployment model uses a PXE/NFS root filesystem, the sysroot can also be copied from the PXE server’s exported root filesystem instead of from a live node.

This is often preferable because the PXE server filesystem is the exact root used by booted nodes.

Example:
```bash
rsync -aHAX --numeric-ids --info=progress2 \
  user@PXEServer:/srv/nfs/ \
  ~/DASPi/images/aperturecomputemodule/sysroot/rootfs
```

## Why the Sysroot Matters
The host compiler targets AArch64, but the linker and build system still need the Pi’s actual libraries and headers.

Without a correct sysroot, common failures include:

- missing ```libopencv_*.so```
- missing ```libcamera```
- incorrect ```pkg-config``` results
- ABI or libstdc++ mismatches
- binaries that link successfully but fail on the target

## Meson Cross File
Meson uses a cross file to describe the target platform, compiler tools, and sysroot.

Example:
```ini
[binaries]
c = 'aarch64-linux-gnu-gcc'
cpp = 'aarch64-linux-gnu-g++'
ar = 'aarch64-linux-gnu-ar'
strip = 'aarch64-linux-gnu-strip'
pkgconfig = 'pkg-config'

[host_machine]
system = 'linux'
cpu_family = 'aarch64'
cpu = 'aarch64'
endian = 'little'

[properties]
sys_root = getenv('DASPI_SYSROOT')

pkg_config_libdir = [
  getenv('DASPI_SYSROOT') + '/usr/lib/aarch64-linux-gnu/pkgconfig',
  getenv('DASPI_SYSROOT') + '/usr/lib/pkgconfig',
  getenv('DASPI_SYSROOT') + '/usr/share/pkgconfig'
]

[built-in options]
c_args = ['--sysroot=getenv('DASPI_SYSROOT')/images/aperturecomputemodule/sysroot/rootfs']
cpp_args = ['--sysroot=getenv('DASPI_SYSROOT')/images/aperturecomputemodule/sysroot/rootfs']
c_link_args = ['--sysroot=getenv('DASPI_SYSROOT')/images/aperturecomputemodule/sysroot/rootfs']
cpp_link_args = ['--sysroot=getenv('DASPI_SYSROOT')/images/aperturecomputemodule/sysroot/rootfs']
```

## Building
Example Meson setup:
```bash
meson setup ~/DASPi/software/build/aperturecomputemodule \
  ~/DASPi/software/src \
  --cross-file ~/DASPi/software/src/aperturecomputemodule/cross-aperturecomputemodule.txt \
  -Dbuild_aperturecomputemodule=true \
  -Dbuild_computemodule=false
```
Compile with:
```bash
meson compile -C ~/DASPi/software/build/aperturecomputemodule
```
Or with a project build script:
```bash
~/DASPi/software/scripts/build-aperturecomputemodule.sh
```

## Deployment
Once built, the resulting binary can be copied into the target root filesystem used by the node.

Example:
```bash
scp ~/DASPi/software/build/aperturecomputemodule/aperturecomputemodule/aperturecomputemodule \
  user@PXEServer:/srv/nfs/opt/daspi/
```
If the nodes boot from /srv/nfs, updating files there updates what the nodes run on next boot.

For quick iteration, it is often enough to copy only the rebuilt binary rather than rebuilding the full image.

## Recommended Workflow
1. edit source on the host machine
1. cross-compile using the AArch64 toolchain
1. copy the rebuilt binary to the PXE/NFS root
1. reboot or restart the service on the Pi node
1. verify runtime behavior on target hardware

This workflow keeps development fast while still testing against the real Raspberry Pi runtime environment.

## Common Problems
```error while loading shared libraries```

Usually means the binary was deployed successfully, but the target runtime is missing a required shared library or the library search path is wrong.

Check with:
```bash
ldd /opt/daspi/aperturecomputemodule
```
```pkg-config``` finds host libraries instead of target libraries

Usually means ```PKG_CONFIG_LIBDIR``` or the Meson cross file is not pointing at the sysroot pkg-config directories.

## Linker errors involving ABI or ```GLIBCXX```

Usually indicates a mismatch between:

- host cross-compiler version
- target libstdc++
- libraries inside the sysroot

In general, the sysroot and cross toolchain should be kept compatible with the deployed Raspberry Pi userspace.

## Build works, runtime fails

Cross-compilation only guarantees successful build/link against the sysroot. It does not guarantee that the deployed target exactly matches that sysroot. Refresh the sysroot if the target image has changed.

## Notes for DASPi

For DASPi development, the sysroot should preferably be taken from the same environment that the ```aperturecomputemodule``` nodes actually boot from.

If the nodes PXE boot from the PXE server, then /srv/nfs is effectively the target root filesystem and is the best source of truth for:

installed libraries
- OpenCV
- libcamera
- service files
- runtime paths

This helps avoid host/target drift and keeps cross-compilation aligned with deployment.

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
