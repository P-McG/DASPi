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

- target headers
- target shared libraries
- pkg-config metadata
- libcamera / OpenCV / other runtime dependencies

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
