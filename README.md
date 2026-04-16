# DASPi
Distributed Aperture System(DAS) for Raspberry Pi Compute Modules

**Very-early work-in-progress**

## Overview
DASPi is a distributed multi-camera system that captures, streams, and stitches frames across multiple Raspberry Pi nodes using UDP.

## Architecture
- PXE Boot Server (central node)
- aperturecomputemodule (capture + transmit)
- computemodule (receive + stitch)

## Quick Start

### 1. Build
./software/scripts/build-aperturecomputemodule.sh

### 2. Deploy to PXE Server
rsync -av build/ user@pxeserver:/srv/nfs/

### 3. Boot Nodes
Power on compute modules (PXE boot)

### 4. Run
System starts automatically via systemd

## Documentation
- [Build & Cross Compilation](docs/build.md)
- [PXE Setup](docs/pxe.md)
- [Camera Pipeline](docs/pipeline.md)
- [UDP Protocol](docs/udp.md)
- [Troubleshooting](docs/troubleshooting.md)

## Quick Debug

- No SSH? → [SSH Troubleshooting](docs/troubleshooting.md#ssh)
- PXE not booting? → [PXE Setup](docs/pxe.md)
- Build failing? → [Build Guide](docs/build.md)
- Bad frames? → [Pipeline Debugging](docs/pipeline.md)

## Repo Structure
```
/software
/images
/docs
```
## License
MIT license
