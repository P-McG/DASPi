# DASPi - troubleshooting

Hard-earned fixes here:
- sudo: unable to resolve host
- missing libopencv_core.so
- invalid magic number
- pixel format mismatch (SBGGR10 vs SBGGR16)
- rsync permission issues
- No response to ```ping <aperturecomputemodule>```
- `apt update` Fails (No Internet from PXE Nodes)
- ERROR: Malformed machine file: Source contains parsing errors:[line  7]: ']\n'

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

## invalid magic number
During UDP frame reception, the system reports:

```bash
Invalid magic number received
```

### What the Magic Number Is

The magic number is a fixed constant placed at the start of each UDP packet (e.g., in ```FrameHeader```). It is used to:

validate that the packet belongs to the DASPi protocol
detect corruption or misalignment
quickly reject invalid or unexpected data
### Root Cause

In DASPi, this issue was **not due to the magic number changing**, but due to **incorrect parsing of incoming data**.

The most common causes observed:

1. Struct Misalignment / Packing Issues

The receiver is interpreting the wrong bytes as the header.

Caused by:

mismatched struct layout between sender and receiver
missing ```#pragma pack``` or inconsistent alignment
differences in 32-bit vs 64-bit compilation

2. Offset Errors in Buffer Parsing
The header is not being read from the correct position in the buffer.

Examples:
- reading after partial data
- incorrect pointer arithmetic
- reusing buffers without resetting offsets

3. Endianness Mismatch
Less common, but possible if:
- sender and receiver interpret byte order differently

4. Packet Fragmentation / Partial Reads
UDP packets are assumed to be whole, but:

- buffer size too small
- truncated recvfrom
- mixing multiple packets in one buffer

5. Wrong Data Source

Receiver is reading:
- non-DASPi traffic
- stale or uninitialized memory

### Key Insight (from debugging)

The magic number itself was correct — the receiver was simply **not aligned to the start of the packet**.

This means:
- The protocol is fine
- The parsing logic is incorrect
This typically appears repeatedly and results in frames being rejected or dropped.
### Fix / Checklist
1. Validate struct consistency
Ensure ```FrameHeader``` is identical on sender and receiver:
```c++
static_assert(sizeof(FrameHeader) == EXPECTED_SIZE);
```
2. Enforce packing if needed
```c++
#pragma pack(push, 1)
struct FrameHeader {
    uint32_t magic;
    ...
};
#pragma pack(pop)
```

3. Verify buffer alignment before casting
```c++
auto* header = reinterpret_cast<const FrameHeader*>(buffer.data());
```
Ensure ```buffer.data()``` points to the start of a full packet.

4. Check receive size
```c++
ssize_t bytes = recvfrom(...);

if (bytes < sizeof(FrameHeader)) {
    // reject packet
}
Print raw bytes for debugging
for (int i = 0; i < 8; ++i) {
    printf("%02X ", buffer[i]);
}
```
Confirm the expected magic number appears at byte 0.

6. Ensure sender and receiver agree on format
- same compiler
- same struct definition
- same packing rules

### Impact
- Frames are dropped early
- Causes apparent data loss or stalled pipelines
- Can lead to misleading debugging (looks like protocol failure)

### Notes (DASPi-specific)
- This was frequently encountered during:
    - UDP pipeline refactoring
    - frame reordering implementation
    - buffer handling changes
- The issue often appears intermittently when:
    - multiple threads are involved
    - buffers are reused incorrectly

### Summary

This error is **almost always a parsing/alignment issue**, not a protocol issue.

Focus debugging on:
- buffer offsets
- struct layout
- receive boundaries

## pixel format mismatch (SBGGR10 vs SBGGR16)
### Cause
Pi 5 (with libcamera v0.6) with Pi's global-shutter camera no longer supports Bayer image mode SBGGR10 and will switch upon verify to Bayer image mode SRGGB10_CSI2P/BGGR_PISP_COMP1.
### Fix
Make sure both libcamera and opencv are using the SBGGR16 uncompressed, or RAW, mode.
### Verify
On the ```aperturecomputemodule``` node run 
```bash 
rpicam-hello list-cameras
```
and this will show the default mode of the setup. Running 
```bash 
/opt/daspi/aperturecomputemodule --verbose --clientIp=10.0.2.1 --port=5000
```
will output 
```
INFO RPI pisp.cpp:1485 Sensor: /base/axi/pcie@1000120000/rp1/i2c@88000/imx296@1a - Selected sensor format: 1456x1088-SBGGR10_1X10/RAW - Selected CFE format: 1456x1088-BYR2/RAW
```

## rsync Permission Issues

### Problem

While copying files (e.g., building images or syncing sysroots), `rsync` reports errors such as:

```bash
rsync: [sender] send_files failed to open "...": Permission denied (13)
```
This commonly occurs when syncing system directories or PXE root filesystems.

### Cause

The issue is caused by **insufficient permissions on the source or destination files**.

In DASPi, this typically happens when:
- copying from system-managed directories (e.g., /var/lib, /srv/nfs)
- syncing a full root filesystem (sysroot creation)
- files are owned by root and not readable by the current user
- special files (e.g., sockets, device nodes) are included

### Key Insight (from debugging)

The failure is not with rsync itself — it is correctly refusing access to restricted files.

Most of the time:
- the files are **not needed** for your use case, or
- the operation simply needs **elevated privileges**

## Fix Options
### Option 1: Use sudo (most common)
```bash
sudo rsync -aHAX --numeric-ids \
  source/ destination/
```
Use this when copying:
- full root filesystems
- PXE ```/srv/nfs``` directories
- sysroots

### Option 2: Exclude problematic directories

When copying a root filesystem, exclude system paths:
```bash
rsync -aHAX --numeric-ids \
  --exclude=/dev \
  --exclude=/proc \
  --exclude=/sys \
  --exclude=/tmp \
  --exclude=/run \
  --exclude=/mnt \
  --exclude=/media \
  source/ destination/
```
These directories are:
- runtime-generated
- not required in a sysroot
- frequent sources of permission errors

### Option 3: Limit scope

Instead of copying ```/```, copy only required paths:
```bash
rsync -a \
  /usr/include \
  /usr/lib \
  destination/
```
Useful for:
- minimal sysroot builds
- faster iteration

## PXE / DASPi-Specific Notes
- When using a PXE server, ```/srv/nfs`` acts as the **canonical root filesystem**
- Permission issues are common when:
    - editing files locally and syncing back
    - mixing root and non-root operations

Recommended workflow:
- use ```sudo rsync``` when modifying ```/srv/nfs```
- ensure consistent ownership inside the PXE root
- avoid partial copies of system directories

## Verification

After syncing, verify:
```bash
ls -l destination/path
```
And for binaries:
```bash
ldd /opt/daspi/aperturecomputemodule
```
## Impact
- Missing files in sysroot → build failures
- Missing libraries → runtime errors
- Incomplete PXE root → boot or service failures

## Summary

This issue is expected when working with system files.

Fix by:
- using ```sudo``` when appropriate
- excluding runtime/system directories
- copying only what is needed

## No Ping Response (Priming the aperturecomputemodule Node)

### Problem

The PXE server or laptop cannot ping the `aperturecomputemodule` node:

```bash
ping <node-ip>
# → no response / timeout
```
This typically happens right after boot, especially in PXE environments.

### What’s Going On

In DASPi, a non-responsive ping during early boot is often not a hard failure, but a timing / initialization issue.

We observed that the node may not respond to ICMP (ping) until its network stack and services are fully initialized.

### Key Insight

In DASPi, the PXEServer provides DHCP, TFTP, and NFS services, but does not necessarily act as a network gateway.

The default gateway must be set to the device that provides internet access and routing (e.g., the laptop at 10.0.2.1), not simply the DHCP server.

Assigning the PXEServer as the gateway without enabling NAT/forwarding will result in nodes being unable to reach external networks.

### Root Causes
#### 1. NIC Not Initialized (First Boot Requirement) ⭐

On Raspberry Pi (especially Pi 5 / Compute Module setups), the NIC firmware must be initialized before proper network operation.

The MAC address is provisioned via the bootloader/ROM during initial boot.

If the board has **never been booted with an SD card**, the NIC may not:
- initialize correctly
- expose a valid MAC address
- participate properly in DHCP / PXE

### Required Step

Boot the node **once using a standard Raspberry Pi OS SD card**:
1. Insert SD card with Raspberry Pi OS
2. Boot the device fully
3. Allow system to initialize (no special configuration required)
4. Power down
5. Remove SD card
6. Retry PXE boot

This ensures:
- NIC firmware is initialized
- MAC address is assigned and stable
- DHCP/PXE works reliably

#### 2. Network Not Fully Initialized
- Interface is up, but:
    - IP not yet assigned (DHCP delay)
    - routing not configured
    - link negotiation still in progress
#### 3. PXE Boot Timing

During PXE boot:
- kernel loads
- rootfs mounts over NFS
- services start
ICMP may fail during this window even though boot is progressing normally.

#### 4. ARP Not Established (“Priming” Effect)
The first successful communication often occurs **only after outbound traffic from the node**.

We observed that:

The node may not respond to ping until it has initiated network activity (e.g., sending UDP frames).

This is effectively **ARP priming**:
- the server does not yet have the node’s MAC/IP mapping
- once the node sends traffic, the mapping is established
- ping then begins to work

#### 5. Wrong Interface / Network Segment
- incorrect subnet (e.g., 10.0.2.x)
- wrong interface used on host
- cabling or switch issues

### Key Insight (from debugging)

Lack of ping response does not necessarily mean the node is down.

In DASPi, nodes were often:
- booted successfully
- running services
- sending UDP data
…while still not responding to ping.

### Priming the Node

To “wake up” network communication:
#### Option 1: Wait
Allow 10–30 seconds after boot for full initialization.

#### Option 2: Trigger Outbound Traffic
Start the capture/transmit process on the node:
- aperturecomputemodule sending UDP frames

This often causes:
- ARP table population
- immediate ping success

#### Option 3: Ping from Node (if accessible)
```bash
ping <PXEServer-IP>
```
#### Option 4: Check ARP Table on Server
```bash
arp -n
```
Verify the node’s MAC/IP mapping appears.

### Verification
```bash
ping <node-ip>
```
Ping should succeed once the node is initialized and/or primed.

### PXE / DASPi-Specific Notes
- Nodes booting from `/srv/nfs` may appear “offline” briefly
- DHCP + NFS + service startup introduces delays
- A one-time SD card boot may be required for NIC initialization
- UDP pipelines may function even when ICMP does not

### Impact
- Misleading debugging signal (appears offline)
- Delays in verifying node availability
- Can be confused with network misconfiguration
### Summary
In DASPi, failed ping during early boot is usually due to:
- NIC not initialized (first boot requirement)
- PXE timing delays
- ARP not yet established
If the node:
- has been booted once via SD card
- eventually sends UDP data
then networking is functioning — it just needs to be **initialized and primed**.

## `apt update` Fails (No Internet from PXE Nodes)

### Problem

PXE-booted nodes cannot access the internet:

```bash
ping 8.8.8.8

# → 100% packet loss
sudo apt update

# → fails / cannot reach repositories
```
### Cause

The node has a valid local network configuration (DHCP, PXE, NFS), but **no valid internet gateway**.

In DASPi, this commonly occurs when:
- dnsmasq assigns the PXEServer (10.0.2.2) as the default gateway:
    ```default via 10.0.2.2```
but the PXEServer does **not** provide internet routing (no NAT / no upstream connection)

Result:
Traffic reaches the PXEServer and stops — no path to the internet.

### Key Insight
PXE boot only requires local networking. Internet access is a separate requirement.

Even if:
- PXE boot works
- NFS root mounts
- services run

…the node will still fail to reach external networks unless a valid gateway exists.

### Fix (DASPi Setup)

Use the **laptop (**10.0.2.1**) as the network gateway**, since it already has internet access.

#### 1. Update `dnsmasq` on PXEServer
Edit:
```bash
sudo nano /etc/dnsmasq.conf
```
Change:
```ini
dhcp-option=3,10.0.2.2
```
to:
```ini
dhcp-option=3,10.0.2.1
```
(Optional but recommended for DNS):
```ini
dhcp-option=6,8.8.8.8
```
#### 2. Restart `dnsmasq`
```bash
sudo systemctl restart dnsmasq
```
#### 3. Reboot node
```bash 
sudo reboot
```
#### 4. Verify
On the node:
```bash
ip route
```
Expected:
```bash
default via 10.0.2.1 dev eth0
```
Then test:
```bash
ping -c 3 8.8.8.8
ping -c 3 deb.debian.org
sudo apt update
```
### Quick Test (Without Reboot)

Temporarily override the gateway:
```bash
sudo ip route replace default via 10.0.2.1
```
### Requirements
- Laptop (10.0.2.1) must have:
    - internet connectivity
    - IP forwarding enabled
    - NAT configured (MASQUERADE)

### Summary

In DASPi, PXEServer is responsible for:
- DHCP
- TFTP
- NFS
…but **not necessarily routing**.

Assigning the correct gateway (laptop) restores internet access for PXE nodes and allows apt to function normally.

## ERROR: Malformed machine file: Source contains parsing errors:[line  7]: ']\n'
### Problem
The cross compilation file is malformed and needs fixed.
### Fix
Put the `]\n` on the previous line.
