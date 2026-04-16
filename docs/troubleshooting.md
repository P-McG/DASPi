# DASPi - troubleshooting

Hard-earned fixes here:
- sudo: unable to resolve host
- missing libopencv_core.so
- invalid magic number
- pixel format mismatch (SBGGR10 vs SBGGR16)
- rsync permission issues

## sudo: unable to resolve host
### Cause

This occurs when the system’s hostname is not properly mapped in ```/etc/hosts```.

```sudo``` attempts to resolve the current hostname. If the hostname is missing or mismatched between:

- ```/etc/hostname```
- ```/etc/hosts```

then name resolution fails, triggering this warning.

This commonly happens in DASPi when:

- cloning or modifying images
- changing hostnames
- PXE boot environments where hostnames are reassigned
- multiple nodes share the same default hostname
### Fix
Check current hostname:
```bash
hostname
```
Ensure it exists in ```/etc/hosts```:
```bash
sudo nano /etc/hosts
```
Add or correct the line:
```127.0.1.1   <hostname>```
Example:
```127.0.1.1   PXEServer```
### Verify
```bash
sudo ls
```

The warning should no longer appear.

### Notes (DASPi-specific)
- In PXE setups, hostnames may be assigned dynamically via DHCP (dnsmasq)
- If DHCP assigns hostnames, ensure:
    - ```/etc/hosts``` is updated accordingly, or
    - hostname resolution is handled consistently across nodes
- Avoid duplicate hostnames across multiple nodes unless intentionally managed
- If using static hostnames in images, ensure each node gets a unique hostname at boot
### Impact

This issue does not usually break functionality, but:

- clutters logs and terminal output
- can indicate deeper hostname/network misconfiguration
- may affect services relying on proper hostname resolution

## pixel format mismatch (SBGGR10 vs SBGGR16)
### Cause
Pi 5 (with libcamera v0.6) with Pi's global-shutter camera no longer supports Bayer image mode SBGGR10 and will switch upon verify to Bayer image mode SRGGB10_CSI2P/BGGR_PISP_COMP1.
### Fix
Make sure both libcamera and opencv are using the SBGGR16 uncompressed, or RAW, mode.
### Verify
On the ```aperturecomputemodule``` node run 
```bash rpicam-hello list-cameras```
and this will show the default mode of the setup. Running 
```bash /opt/daspi/aperturecomputemodule --verbose --clientIp=10.0.2.1 --port=5000```
will output ```INFO RPI pisp.cpp:1485 Sensor: /base/axi/pcie@1000120000/rp1/i2c@88000/imx296@1a - Selected sensor format: 1456x1088-SBGGR10_1X10/RAW - Selected CFE format: 1456x1088-BYR2/RAW```
