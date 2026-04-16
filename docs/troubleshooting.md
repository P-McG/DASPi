# DASPi - troubleshooting

Hard-earned fixes here:
- sudo: unable to resolve host
- missing libopencv_core.so
- invalid magic number
- pixel format mismatch (SBGGR10 vs SBGGR16)
- rsync permission issues

## sudo: unable to resolve host
Cause

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
-may affect services relying on proper hostname resolution
