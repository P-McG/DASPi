# First-Time Bring-Up Checklist (aperturecomputemodule Node) for PXE

This checklist ensures a new Raspberry Pi node is properly initialized for PXE boot and DASPi operation.

---

### 1. Initial SD Card Boot (Required) ⭐

Before attempting PXE boot, the node must be booted once using an SD card.

Why:
- Initializes NIC firmware
- Ensures a valid MAC address is provisioned
- Enables reliable DHCP/PXE behavior

Steps:

1. Flash Raspberry Pi OS Lite (64-bit) to an SD card
2. Insert SD card into the node
3. Power on and allow full boot
4. (Optional) Log in via SSH or console
5. Power down the node
6. Remove the SD card

---

### 2. Record MAC Address

Boot the node (SD card or PXE) and obtain its MAC address:

```bash
ip link
```
Look for:
```bash
eth0: ... link/ether AA:BB:CC:DD:EE:FF
```

### 3. Configure DHCP (PXE Server)
Add a static lease in `dnsmasq.conf`:
```ini
dhcp-host=AA:BB:CC:DD:EE:FF,10.0.2.10
```
This ensures:
- consistent IP assignment
- easier debugging and node identification

Restart dnsmasq:
```bash
sudo systemctl restart dnsmasq
```

### 4. Verify PXE Boot Files
Ensure required files exist on the PXE server:
```
/srv/tftp/
  ├── start4.elf
  ├── config.txt
  ├── cmdline.txt
  ├── kernel_2712.img

/srv/nfs/
  └── (root filesystem)
```
### 5. Boot via PXE
1. Ensure SD card is removed
2. Connect Ethernet
3. Power on the node

Monitor PXE server logs:
```bash
journalctl -u dnsmasq -f
```
You should see TFTP activity (e.g., kernel requests).

### 6. Verify Network Connectivity
From the PXE server:
```bash
ping 10.0.2.10
```
If ping fails:
- wait 10–30 seconds
- see “No Ping Response” section

### 7. Verify Node Activity
Even if ping fails, confirm the node is active:
- check for UDP traffic
- check logs on PXE server
- verify service startup (if applicable)

### 8. Deploy DASPi Binary

Copy the latest build:
```bash
scp aperturecomputemodule \
  user@PXEServer:/srv/nfs/opt/daspi/
```

### 9. Verify Runtime

On boot, ensure:
- 'aperturecomputemodule' starts
- no missing libraries:
```bash
ldd /opt/daspi/aperturecomputemodule
```

10. Common First-Time Issues
- ❌ No ping → NIC not initialized or ARP not primed
- ❌ No DHCP lease → MAC not configured in dnsmasq
- ❌ TFTP failures → missing /srv/tftp files
- ❌ Runtime errors → missing libraries in /srv/nfs

### Summary
A successful bring-up requires:
- one-time SD card boot (NIC initialization)
- correct MAC/DHCP configuration
- valid PXE server setup
- synchronized /srv/nfs root filesystem

Once completed, nodes should boot reliably via PXE and integrate into the DASPi pipeline.
