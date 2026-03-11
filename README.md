# DASPi
Distributed Aperture System (DAS) using the Raspberry Pi.
DASPi consists of n-cameras mounted around the physical object in such a way as to provide unobstructed spherical (4π steradian) coverage.

Very early work in progress

# DASPi
Distributed Aperture System (DAS) using the Raspberry Pi.
DASPi consists of n-cameras mounted around the physical object in such a way as to provide unobstructed spherical (4π steradian) coverage.

Very early work in progress




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
rsync -avz $USER@aperturecomputemodule000:/lib ~/aperturecomputemodule-sysroot/
rsync -avz $USER@aperturecomputemodule000:/usr ~/aperturecomputemodule-sysroot/

Set up the cross compiling sources for the computemodule000. This needs to be performed every major change, or update, to the computemodules.
mkdir -p ~/computemodule-sysroot
rsync -avz $USER@computemodule000:/lib ~/computemodule-sysroot/
rsync -avz $USER@computemodule000:/usr ~/computemodule-sysroot/

To setup meson for aperturecomputemodule 
export SYSROOT="$HOME/aperturecomputemodule-sysroot"
export PKG_CONFIG_LIBDIR="$SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig:$SYSROOT/usr/share/pkgconfig"
meson setup build --cross-file cross-aperturecomputemodule.txt

To cross compile aperturecomputemodule
meson compile
cp ../build/aperturecomputemodule ../bin
~/DASPi/src/aperturecomputemodule/distribute_and_run_aperturecomputemodule_code.sh

To setup meson for computemodule 
export SYSROOT="$HOME/computemodule-sysroot"
export PKG_CONFIG_LIBDIR="$SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig:$SYSROOT/usr/share/pkgconfig"
rm -rf build
meson setup build --cross-file cross-computemodule.txt

To cross compile computemodule
~/DASPi/src/computemodule/distribute_and_run_computemodule_code.sh

To copy output from the computemodule
scp computemodule000:~/DASPi/src/computemodule/bin/output-10.0.2.3_0.bayer ~/output-10.0.2.3_0.bayer
scp computemodule000:~/DASPi/src/computemodule/bin/output-10.0.3.3_0.bayer ~/output-10.0.3.3_0.bayer


To display the output.
ffplay -f rawvideo -pixel_format bayer_rggb16le -video_size 1456x1088 ~/output-10.0.2.3_0.bayer
