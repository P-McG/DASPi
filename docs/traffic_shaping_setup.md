# DASPi: ComputeModule and ApertureComputeModule Traffic Shaping

## Overview

This document records the Linux networking configuration tested with four Raspberry Pi 5 **ApertureComputeModules** streaming camera frames to a Dell Precision 3591 **ComputeModule** running Ubuntu 24.04.

**Tested outcome:** After applying a 300 Mbit/s Token Bucket Filter (TBF) to each Raspberry Pi's outgoing Ethernet traffic, all four receiver streams ran at approximately **30 frames/s** in the sampled intervals. The Linux TBF queues reported **zero dropped packets**, and the Dell's UDP counters reported **zero input and receive-buffer errors**.

The camera transmission shaper runs **on the Raspberry Pis**, not on the Dell. The Dell uses a 2.5 GbE link and configured receive-buffer limits.

> **Scope:** These are the working settings for the approximately 1,025,996-byte UDP frame payload used in the test. They are not automatically suitable for larger payloads or additional cameras. The persistence instructions below are the intended setup; successful operation *after reboot* should be verified separately.

## 1. Network topology

```text
                      Dell Precision 3591
                    ComputeModule (Ubuntu)
                           10.0.2.1
                    USB Realtek RTL8156 NIC
                     2.5 GbE (full duplex)
                               |
                      2.5 GbE Ethernet
                               |
                     MokerLink 2G08110GS
                         Ethernet switch
                          /  /  \  \
                         /  /    \  \
                   1GbE links to four Raspberry Pi 5 nodes
                       |     |     |     |
                     .4    .5    .6    .7
                     000   001   002   003

              Each Raspberry Pi: eth0 TBF @ 300 Mbit/s
```

| Device | IP address | Network interface | Role |
|---|---|---|---|
| Dell Precision 3591 | `10.0.2.1/24` | `enxe8cf830e19f9` | UDP receiver / ComputeModule |
| ApertureComputeModule000 | `10.0.2.4` | `eth0` | Camera sender |
| ApertureComputeModule001 | `10.0.2.5` | `eth0` | Camera sender |
| ApertureComputeModule002 | `10.0.2.6` | `eth0` | Camera sender |
| ApertureComputeModule003 | `10.0.2.7` | `eth0` | Camera sender |

## 2. ComputeModule configuration (Dell Precision 3591)

### 2.1 Ethernet adapter

The USB 2.5 GbE adapter was detected with the in-kernel Realtek `r8152` driver.

| Setting | Observed value |
|---|---|
| Linux interface | `enxe8cf830e19f9` |
| NetworkManager profile | `DASPi-2.5GbE` |
| IPv4 address | `10.0.2.1/24` |
| Ethernet speed | `2500Mb/s` |
| Duplex | `Full` |
| Driver | `r8152` |
| USB connection speed | `5000M` |
| Queue discipline | `fq_codel` (outgoing) |

Verify:

```bash
nmcli device status
ip -4 addr show enxe8cf830e19f9
ip route get 10.0.2.4
sudo ethtool enxe8cf830e19f9
lsusb -t
```

The route to `10.0.2.4` should use `enxe8cf830e19f9` with source address `10.0.2.1`.

**Do not apply the camera TBF rule to the Dell's interface.** That rule is intended to shape camera packets leaving each Raspberry Pi.

### 2.2 Receive-side Linux kernel settings

The Dell reported these active settings:

| Kernel parameter | Tested value | Meaning |
|---|---:|---|
| `net.core.rmem_max` | `134217728` (128 MiB) | Maximum permitted receive socket buffer |
| `net.core.rmem_default` | `33554432` (32 MiB) | Default receive socket buffer |
| `net.core.netdev_max_backlog` | `1000` | Network input backlog setting (unchanged) |
| `net.core.wmem_max` | `212992` | Send-buffer maximum (unchanged) |
| `net.core.wmem_default` | `212992` | Default send buffer (unchanged) |

Apply the two receive-buffer values immediately:

```bash
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=33554432
```

Make the two values persistent across reboots:

```bash
sudo tee /etc/sysctl.d/90-daspi-udp.conf > /dev/null <<'EOF_SYSCTL'
net.core.rmem_max = 134217728
net.core.rmem_default = 33554432
EOF_SYSCTL

sudo sysctl -p /etc/sysctl.d/90-daspi-udp.conf
```

Verify:

```bash
sysctl net.core.rmem_max net.core.rmem_default net.core.netdev_max_backlog
```

> **Important:** `rmem_max` establishes a ceiling. It does not itself allocate a 128 MiB buffer to each UDP socket; the ComputeModule application must request an appropriate receive buffer if it needs one.

## 3. ApertureComputeModule configuration (Raspberry Pi 5)

### 3.1 Tested transmission-shaping parameters

| Parameter | Value |
|---|---|
| Device | `eth0` |
| Queue discipline | TBF — Token Bucket Filter |
| Rate | `300mbit` |
| Burst | `64kb` |
| Latency | `100ms` |

The `rate` limits outgoing packets on each Pi; `burst` permits short bursts, and `latency` influences the queue capacity. The objective is to spread UDP traffic over time instead of sending each frame in a large line-rate burst.

### 3.2 Apply shaping temporarily to all four Pis

Run on the **Dell**:

```bash
for ip in 10.0.2.{4..7}; do
    echo "===== Configuring $ip ====="
    ssh -t daspi@"$ip" \
      'sudo /usr/sbin/tc qdisc replace dev eth0 root tbf \
       rate 300mbit burst 64kb latency 100ms'
done
```

This replaces the previously observed `pfifo_fast` root queue on each Raspberry Pi. `tc` was present at `/usr/sbin/tc` even when the simple command `tc` was not found in a noninteractive SSH session.

### 3.3 Verify the TBF settings and counters

```bash
for ip in 10.0.2.{4..7}; do
    echo "===== $ip ====="
    ssh daspi@"$ip" '/usr/sbin/tc -s qdisc show dev eth0'
done
```

Expected example:

```text
qdisc tbf ... root ... rate 300Mbit burst 64Kb lat 100ms
 Sent ... bytes ... pkt (dropped 0, overlimits ... requeues 0)
 backlog ...
```

Interpretation:

- `dropped 0`: No packets discarded by that TBF queue over the lifetime of its counters.
- `overlimits`: Normal for rate shaping; these reflect rate enforcement and are **not** equivalent to drops.
- `backlog`: Bytes/packets waiting in the shaping queue. It can be nonzero while cameras are streaming.

### 3.4 Start or check the camera services

Start on all four nodes:

```bash
for ip in 10.0.2.{4..7}; do
    ssh -t daspi@"$ip" 'sudo systemctl start aperturecomputemodule'
done
```

Verify:

```bash
for ip in 10.0.2.{4..7}; do
    echo "===== $ip ====="
    ssh daspi@"$ip" 'systemctl is-active aperturecomputemodule'
done
```

Use `systemctl` for services; `sysctl` is for kernel parameters.

## 4. Make Raspberry Pi shaping persistent with systemd

The four Raspberry Pis are configured to use a shared NFS root filesystem. A service created under `/etc/systemd/system` in that shared root should therefore be visible to the four nodes. Check the actual mount layout before assuming a change is shared.

### 4.1 Create the shaping service

SSH to one Pi (for example `10.0.2.4`):

```bash
ssh daspi@10.0.2.4
```

Create `/etc/systemd/system/daspi-tx-shaping.service`:

```bash
sudo tee /etc/systemd/system/daspi-tx-shaping.service > /dev/null <<'EOF_SERVICE'
[Unit]
Description=DASPi 300Mb/s Camera Transmission Shaping
Wants=network-online.target
After=network-online.target
Before=aperturecomputemodule.service

[Service]
Type=oneshot
ExecStart=/usr/sbin/tc qdisc replace dev eth0 root tbf rate 300mbit burst 64kb latency 100ms
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF_SERVICE
```

Enable and activate:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now daspi-tx-shaping.service
```

### 4.2 Ensure the camera service starts after the shaper

Create a drop-in dependency for `aperturecomputemodule.service`:

```bash
sudo mkdir -p /etc/systemd/system/aperturecomputemodule.service.d

sudo tee /etc/systemd/system/aperturecomputemodule.service.d/10-tx-shaping.conf > /dev/null <<'EOF_DEPENDENCY'
[Unit]
Requires=daspi-tx-shaping.service
After=daspi-tx-shaping.service
EOF_DEPENDENCY
```

Return to the Dell:

```bash
exit
```

Reload systemd on all four nodes (each has its own running systemd manager, even when the service files are shared):

```bash
for ip in 10.0.2.{4..7}; do
    ssh -t daspi@"$ip" 'sudo systemctl daemon-reload'
done
```

The enabling symlink should propagate through the shared root if the nodes truly share `/etc/systemd/system`. Confirm this rather than assuming it.

### 4.3 Reboot verification

Reboot **one Pi first** and confirm the rule returns:

```bash
ssh daspi@10.0.2.4 'systemctl is-active daspi-tx-shaping.service'
ssh daspi@10.0.2.4 '/usr/sbin/tc -s qdisc show dev eth0'
```

The service should be `active`, and the queue should show `tbf ... rate 300Mbit`. Only after that test should the other nodes be rebooted and checked.

### 4.4 Undo the temporary shaper if necessary

To remove the live root TBF queue on each Raspberry Pi:

```bash
for ip in 10.0.2.{4..7}; do
    ssh -t daspi@"$ip" 'sudo /usr/sbin/tc qdisc del dev eth0 root'
done
```

If the persistent service has been installed, also disable it (and remove or adjust the camera-service dependency) before expecting the change to survive a reboot.

## 5. Measured results

### 5.1 Receiver performance: before versus after shaping

| Measurement | Before 300 Mb/s TBF | After 300 Mb/s TBF |
|---|---:|---:|
| Completed receive FPS per camera (sampled) | ~6–9 | ~30 |
| Module 000 frame-loop time | 123.74 ms | 28.05 ms |
| Module 002 frame-loop time | 120.06 ms | 28.83 ms |
| Module 002 publish FPS | 7.96 | 28.76 |
| Module 003 publish FPS | 6.62 | 31.20 |
| Module 003 completed RX FPS | 6.00 | 31.24 |

In the final set of **five RX summary intervals per camera**, the approximate per-camera averages were:

| Module | Mean RX FPS | Sum of `rxDropOld` in those intervals |
|---|---:|---:|
| 000 | 29.91 | 1 |
| 001 | 29.96 | 1 |
| 002 | 30.01 | 0 |
| 003 | 30.04 | 1 |

`rxTimeout` and `rxFail` were zero in those reported summary intervals. The exact meaning of `rxDropOld` should be confirmed in the C++ receiver implementation before equating it to a complete frame loss.

### 5.2 Extended TBF and UDP counters

In a subsequent set of cumulative TBF statistics, **each** Pi had transmitted approximately 2.58 TB with **zero TBF drops**:

| Module | Approx. transmitted data | TBF drops | Snapshot backlog |
|---|---:|---:|---:|
| 000 | 2.583 TB | 0 | 87,962 B |
| 001 | 2.583 TB | 0 | 96,614 B |
| 002 | 2.584 TB | 0 | 90,846 B |
| 003 | 2.584 TB | 0 | 0 B |

The Dell also reported:

```text
UdpInErrors       0
UdpRcvbufErrors   0
```

These are kernel and qdisc counters, not an independent proof that every camera frame was delivered. Continuous application-level 30 fps should still be verified in longer runs.

### 5.3 Throughput and stitch loop

The Dell's earlier interface-bandwidth test showed approximately **940 Mb/s** of sustained aggregate receiving traffic. That was measured before the final shaped-run results, so it should not be presented as a separately measured shaped-run throughput.

The application's `[Stitch timing]` loop reported approximately **69–74 loops/s**, but `stitch_ms=0`. That timing is **not proof** of 69–74 unique newly stitched panoramas per second; the update and stitch stages need further investigation, especially because overlap regions were previously missing.

## 6. Monitoring commands

**Dell: UDP receive errors**

```bash
nstat -az UdpInErrors UdpRcvbufErrors
```

**Dell: Ethernet receive errors/drops**

```bash
ip -s link show enxe8cf830e19f9
```

**Dell: Live received bitrate, five-second intervals**

```bash
IF=enxe8cf830e19f9
while true; do
    before=$(cat "/sys/class/net/$IF/statistics/rx_bytes")
    sleep 5
    after=$(cat "/sys/class/net/$IF/statistics/rx_bytes")
    awk -v a="$before" -v b="$after" 'BEGIN {
        printf "RX: %.1f Mb/s (%.1f MB/s)\n", \
               (b-a)*8/5000000, (b-a)/5000000
    }'
done
```

**Pi: TBF counters**

```bash
for ip in 10.0.2.{4..7}; do
    echo "===== $ip ====="
    ssh daspi@"$ip" '/usr/sbin/tc -s qdisc show dev eth0'
done
```

**Dell: Application receive and stitch timing** (adjust log path if needed)

```bash
grep -a '\[RX summary\]' /tmp/daspi-2p5gbe-test.log | tail -n 20
grep -a '\[Stitch timing\]' /tmp/daspi-2p5gbe-test.log | tail -n 5
```

## 7. Capacity and future changes

At **1,025,996 payload bytes per frame** and **30 fps**:

```text
1,025,996 bytes/frame × 30 frames/s × 8 = 246.24 Mbit/s per camera
```

A 300 Mbit/s TBF cap leaves some room for packet overhead and bursts at that payload size.

At **2,052,012 payload bytes per frame** and **30 fps**:

```text
2,052,012 bytes/frame × 30 frames/s × 8 = 492.48 Mbit/s per camera
```

A 300 Mbit/s TBF would **not** sustain the larger frame size at 30 fps. Review camera payload sizes, packet overhead, queue drops, and the aggregate 2.5 GbE link capacity before expanding the system or changing frame formats.

## 8. Recommended baseline

- **Dell ComputeModule:** 2.5 GbE at `10.0.2.1/24`, `rmem_max=128 MiB`, `rmem_default=32 MiB`; no outgoing TBF limiter needed for camera reception.
- **Each Raspberry Pi ApertureComputeModule:** `eth0` TBF at **300 Mbit/s**, **64 KB burst**, **100 ms latency** for the tested ~1.026 MB frame size.
- **Persistence:** Use `daspi-tx-shaping.service` and the `aperturecomputemodule.service` drop-in; verify actual boot behavior on one Pi before deploying to all.
- **Operational acceptance:** Four completed receiver streams near **30 fps**, no accumulating TBF drops or UDP errors, and acceptable application-level frame continuity.

**Conclusion:** The 2.5 GbE upgrade provided aggregate network capacity; shaping the Raspberry Pi transmissions was associated with a large improvement in the Dell receiver's completed frame rate. Preserve the tested configuration, verify it across reboots, and investigate panorama stitching/overlap separately.
