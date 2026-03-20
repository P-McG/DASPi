# DASPi
## What it is
Distributed Aperture System (DAS) using the Raspberry Pi.
DASPi consists of n-cameras mounted around the physical object in such a way as to provide unobstructed spherical (4π steradian) coverage.

Very early work in progress

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
### USB-hub
USB 2.0 Hub Type-C Multi Safety 10 Ports Powered, with AC Adapter, Aluminum USB Splitter with Cooling Fan - Type C
## Repo layout
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

## Build prerequisites for Compute-Module
### Host file On host
Place in host file:

```[ip_address_here] computemodule000
[ip_address_here] aperturecomputemodule000
.                      .
.                      .
.                      .
[ip_address_here] aperturecomputemoduleNNN```

sudo apt update
sudo apt install -y tmux

## Build prerequisites for Aperture-Compute-Module

### Host file On host
Place in host file:
```[ip_address_here] computemodule000
[ip_address_here] aperturecomputemodule000
.                      .
.                      .
.                      .
[ip_address_here] aperturecomputemoduleNNN```

sudo apt update
sudo apt install -y tmux

### Cross-Compiling Sources for the Compute-Module and Aperture-Compute-Module
Set up the cross compiling sources for the aperturecomputemodule000. 
This needs to be performed every major change, or update, to the aperturecomputemodules. 
Only one of the aperturecomputemodules is needed to be synced if all of the aperturecomputemodules are the same. 

```~/DASPi/software/scripts/update_sysroot.sh```

## Cross-Compiling aperturecomputemodule
```rm -rf ~/DASPi/software/build/aperturecomputemodule

meson setup ~/DASPi/software/build/aperturecomputemodule \
  ~/DASPi/software/src \
  --cross-file ~/DASPi/software/src/aperturecomputemodule/cross-aperturecomputemodule.txt \
  -Dbuild_aperturecomputemodule=true \
  -Dbuild_computemodule=false

meson compile -C ~/DASPi/software/build/aperturecomputemodule

~/DASPi/src/distribute_and_run_aperturecomputemodule_code.sh```

## Cross-compiling computemodule 
```rm -rf ~/DASPi/software/build/computemodule

meson setup ~/DASPi/software/build/computemodule \
  ~/DASPi/software/src \
  --cross-file ~/DASPi/software/src/computemodule/cross-computemodule.txt \
  -Dbuild_aperturecomputemodule=false \
  -Dbuild_computemodule=true

meson compile -C ~/DASPi/software/build/computemodule

~/DASPi/src/distribute_and_run_computemodule_code.sh```
## Deployment
To copy output from the computemodule
```~/DASPi/software/scripts/fetch_files.sh```

## Viewing captured output
```ffplay -f rawvideo -pixel_format bayer_rggb16le -video_size 
1456x1088 ~/output-10.0.2.3_0.bayer```

## GIT Commits Notes:
Sync workflow 
Before working:
git pull
After changes:
git add .
git commit -m "message"
git push

## Current status / roadmap
