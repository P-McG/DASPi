# DASPi – PXE (Preboot Execution Environment)

**DASPi PXE** enables diskless boot and centralized management of Raspberry Pi–based compute nodes within the DASPi system. Using standard **PXE** mechanisms, devices boot over the network instead of relying on local storage such as SD cards.

In a DASPi deployment, a PXE server provides:

- **DHCP services** to assign IP addresses and identify clients
- **TFTP** boot resources (bootloader, kernel, firmware)
- **NFS (or similar) root filesystem** for the operating system

When a compute module powers on, it:
1. Requests network configuration via DHCP
2. Downloads boot files from the PXE server
3. Mounts its root filesystem over the network
4. Launches DASPi services (e.g., aperturecomputemodule) directly from the shared image

This architecture allows:

- **Centralized updates** (modify one image, update all nodes)
- **Stateless nodes** (no SD card management or corruption issues)
- **Rapid scaling** (new nodes join by simply connecting to the network)
- **Consistent environments** across all compute modules

In DASPi, PXE is especially useful for multi-node camera and processing clusters, where synchronized software deployment and reproducibility are critical.

## DASPi - PXE setup:

- dnsmasq config
- /srv/tftp + /srv/nfs
- Boot flow for Pi 5
- Static IP + MAC mapping
- Hostname handling issues

## PXE Configuration (DASPi Network)

This configuration assumes a fixed DASPi network:

| Device                    | IP Address   |
|--------------------------|-------------|
| Laptop                   | 10.0.2.1     |
| PXEServer                | 10.0.2.2     |
| computemodule            | 10.0.2.3     |
| aperturecomputemodule000 | 10.0.2.4     |
| aperturecomputemodule001 | 10.0.2.5     |

All nodes are connected via Ethernet on an isolated network.

---

## dnsmasq Configuration

Edit:

```bash
sudo nano /etc/dnsmasq.conf
```
Use:
```ini
# =========================
# DASPi PXE Server Config
# =========================

# Bind to DASPi Ethernet interface
interface=enp0s31f6
bind-interfaces

# Disable DNS (we only care about DHCP + TFTP)
port=0

# DHCP range (dynamic pool)
dhcp-range=10.0.2.10,10.0.2.50,12h

# Default gateway (optional, isolated network doesn't need it)
dhcp-option=3,10.0.2.2

# DNS server (optional)
dhcp-option=6,10.0.2.2

# =========================
# Static IP Assignments
# =========================
# Replace MAC addresses!

dhcp-host=AA:BB:CC:DD:EE:03,10.0.2.3   # computemodule
dhcp-host=AA:BB:CC:DD:EE:04,10.0.2.4   # aperturecomputemodule000
dhcp-host=AA:BB:CC:DD:EE:05,10.0.2.5   # aperturecomputemodule001

# =========================
# PXE / TFTP
# =========================
enable-tftp
tftp-root=/srv/tftp

# Required for Raspberry Pi PXE
dhcp-boot=bootcode.bin

# =========================
# Logging (VERY useful)
# =========================
log-dhcp
log-queries
```

Restart:
```bash
sudo systemctl restart dnsmasq
```
## TFTP Boot Files (/srv/tftp)

Required files:
```
/srv/tftp/
  ├── start4.elf
  ├── fixup4.dat
  ├── kernel_2712.img
  ├── config.txt
  ├── cmdline.txt
```
## config.txt
```ini
# Enable 64-bit kernel
arm_64bit=1

# Enable UART for debugging (optional)
enable_uart=1

# Boot over network
boot_delay=1
```
## cmdline.txt (CRITICAL)
**Single line only**:
```ini
console=serial0,115200 console=tty1 root=/dev/nfs nfsroot=10.0.2.2:/srv/nfs,vers=4.1,proto=tcp rw ip=dhcp rootwait
```

## Minimal config.txt
```ini
arm_64bit=1
enable_uart=1
boot_delay=1
```
## NFS Root (/srv/nfs)
Export:
```bash
sudo nano /etc/exports
```
```ini
/srv/nfs 10.0.2.0/24(rw,sync,no_subtree_check,no_root_squash)
```
Apply:
```bash
sudo exportfs -ra
sudo systemctl restart nfs-kernel-server
```
## Host Configuration
### Laptop (10.0.2.1)
Set static IP on Ethernet:
```bash
sudo ip addr add 10.0.2.1/24 dev <interface>
sudo ip link set <interface> up
```
### PXEServer (10.0.2.2)
```bash
sudo ip addr add 10.0.2.2/24 dev enp0s31f6
```
Ensure:
- dnsmasq is bound to this interface
- no conflicting DHCP servers exist
## Verification
### 1. Monitor boot
```bash
journalctl -u dnsmasq -f
```
Look for:
- DHCP requests
- TFTP transfers

### 2. Confirm leases
```bash
cat /var/lib/misc/dnsmasq.leases
```
### 3. Ping nodes
```bash
ping 10.0.2.4
ping 10.0.2.5
```
## DASPi-Specific Notes

### DASPi Workflow (this is the real value)
This ties everything together cleanly:
🔁 Build → Deploy → Run Loop

#### Step 1: Build (on laptop)
~/DASPi/software/scripts/build-aperturecomputemodule.sh

#### Step 2: Deploy to PXEServer
Updating binaries here updates all nodes instantly on reboot
```bash
scp ~/DASPi/software/build/aperturecomputemodule/aperturecomputemodule \
  user@10.0.2.2:/srv/nfs/opt/daspi/
```
`/srv/nfs` is the live filesystem for:
    - computemodule
    - aperturecomputemodule nodes

#### Step 3: Restart node
Either:
```bash
sudo reboot
```
OR power cycle

#### Step 4: Watch boot live
```bash
ssh user@10.0.2.2
journalctl -u dnsmasq -f
```
You should see:
- DHCP request
- TFTP transfers
- kernel load

#### Step 5: Verify runtime
```bash
ping 10.0.2.4
```
If not:
see “Priming” section (expected sometimes)

#### 6. Debug checklist (fast)

If something breaks:
##### No DHCP
- wrong interface in dnsmasq
- MAC mismatch
##### TFTP loops / failures
- missing /srv/tftp files
- wrong filenames (kernel_2712.img!)
##### Kernel boots but hangs
- bad `cmdline.txt`
- NFS export wrong
#### App fails to start
```bash
ldd /opt/daspi/aperturecomputemodule
```

---

## Common Pitfalls
- ❌ Wrong interface in dnsmasq → no DHCP
- ❌ MAC mismatch → wrong IP assignment
- ❌ Missing `cmdline.txt` → kernel boot fails
- ❌ NFS export incorrect → boot hangs
- ❌ Node never booted from SD → NIC not initialized

## Summary
This configuration provides:
- deterministic IPs
- centralized boot and filesystem
- consistent DASPi deployment

All nodes should now PXE boot and integrate cleanly into the system.
