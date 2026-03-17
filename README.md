# DASPi
## What it is
Distributed Aperture System (DAS) using the Raspberry Pi.
DASPi consists of n-cameras mounted around the physical object in such a way as to provide unobstructed spherical (4π steradian) coverage.

Very early work in progress

## Repo layout
## Build prerequisites

On host
Place in host file:
[ip_address_here] computemodule000
[ip_address_here] aperturecomputemodule000
.                      .
.                      .
.                      .
[ip_address_here] aperturecomputemoduleNNN

Set up the cross compiling sources for the aperturecomputemodule000. 
This needs to be performed every major change, or update, to the aperturecomputemodules. 
Only one of the aperturecomputemodules is needed to be synced if all of the aperturecomputemodules are the same. 
mkdir -p ~/aperturecomputemodule-sysroot
rsync -a --delete $USER@aperturecomputemodule000:/lib/      ~/aperturecomputemodule-sysroot/lib/
rsync -a --delete $USER@aperturecomputemodule000:/usr/      ~/aperturecomputemodule-sysroot/usr/
rsync -a --delete $USER@aperturecomputemodule000:/opt/      ~/aperturecomputemodule-sysroot/opt/      # if needed
rsync -a --delete $USER@aperturecomputemodule000:/etc/ld.so.conf* ~/aperturecomputemodule-sysroot/etc/

Set up the cross compiling sources for the computemodule000. This needs to be performed every major change, or update, to the computemodules.
mkdir -p ~/computemodule-sysroot
rsync -a --delete $USER@computemodule000:/lib/      ~/computemodule-sysroot/lib/
rsync -a --delete $USER@computemodule000:/usr/      ~/computemodule-sysroot/usr/
rsync -a --delete $USER@computemodule000:/opt/      ~/computemodule-sysroot/opt/      # if needed
rsync -a --delete $USER@computemodule000:/etc/ld.so.conf* ~/computemodule-sysroot/etc/

## Cross-compiling aperturecomputemodule
cd ~/DASPi/software
rm -rf build
meson setup build-aperturecomputemodule \
  --cross-file aperturecomputemodule/cross-aperturecomputemodule.txt \
  -Dbuild_aperturecomputemodule=true \
  -Dbuild_computemodule=false
meson compile -C build-aperturecomputemodule
cp -r ./build-aperturecomputemodule/aperturecomputemodule ./bin-aperturecomputemodule
~/DASPi/software/distribute_and_run_aperturecomputemodule_code.sh  

ninja -C build-aperture

## Cross-compiling computemodule 
meson setup build-computemodule \
  --cross-file computemodule/cross-computemodule.txt \
  -Dbuild_aperturecomputemodule=false \
  -Dbuild_computemodule=true
meson compile -C build-computemodule
cp -r ./build-computemodule/computemodule ./bin-computemodule
~/DASPi/software/distribute_and_run_computemodule_code.sh
## Deployment
To copy output from the computemodule
scp computemodule000:~/DASPi/src/computemodule/bin/output-10.0.2.3_0.bayer ~/output-10.0.2.3_0.bayer
scp computemodule000:~/DASPi/src/computemodule/bin/output-10.0.3.3_0.bayer ~/output-10.0.3.3_0.bayer

## Viewing captured output
ffplay -f rawvideo -pixel_format bayer_rggb16le -video_size 1456x1088 ~/output-10.0.2.3_0.bayer

##GIT Commits Notes:
Sync workflow 
Before working:
git pull
After changes:
git add .
git commit -m "message"
git push

## Current status / roadmap
