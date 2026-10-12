# DASPi — Procedure to Fix Duplicate Raspberry Pi Hostnames

This documents the successful solution for your four Raspberry Pi 5 ApertureComputeModules running Debian Trixie using a shared PXE/NFS root filesystem.

Root cause: Cloud-init was overwriting the hostnames supplied by the PXE boot configuration, setting every Raspberry Pi to `aperturecomputemodule000`.

## 1. Configure unique hostnames through PXE

On PXEServer, edit:

```
sudo nano /srv/tftp/config.txt
```

Ensure the serial-number-specific configuration is present:

```
[cm5]
dtoverlay=dwc2,dr_mode=host

[all]

[0xb269f131]
cmdline=cmdline-b269f131.txt

[0x10d17922]
cmdline=cmdline-10d17922.txt

[0x5337316b]
cmdline=cmdline-5337316b.txt

[0x660c5b82]
cmdline=cmdline-660c5b82.txt

[all]
```

The `[all]` after `[cm5]` ensures the serial-specific settings are not restricted to Compute Module 5 boards.

Each serial-specific `cmdline-XXXXXXXX.txt` file must contain its respective hostname assignment, appended to the existing kernel command line:

| Serial suffix | Required parameter                          |
| ------------- | ------------------------------------------- |
| `b269f131`    | `systemd.hostname=aperturecomputemodule000` |
| `10d17922`    | `systemd.hostname=aperturecomputemodule001` |
| `5337316b`    | `systemd.hostname=aperturecomputemodule002` |
| `660c5b82`    | `systemd.hostname=aperturecomputemodule003` |

Keep each kernel command line on one line and preserve the existing NFS boot settings.

## 2. Disable the old DHCP hostname service

On one Raspberry Pi:

```
sudo systemctl disable --now set-hostname-from-dhcp.service
```

The custom service previously executed:

```
hostnamectl set-hostname $(hostname -f)
```

This could write a common hostname into the shared `/etc/hostname` file.

If the service is already masked, leave it masked. Otherwise, to mask a custom unit stored directly in `/etc/systemd/system`, first move the unit file out of the way:

```
sudo mv /etc/systemd/system/set-hostname-from-dhcp.service \
    /root/set-hostname-from-dhcp.service.bak

sudo systemctl daemon-reload
sudo systemctl mask set-hostname-from-dhcp.service
```

## 3. Stop cloud-init from overwriting hostnames

This was the decisive fix.

Cloud-init was configured with:

```
preserve_hostname: false
```

And `/boot/firmware/user-data` contained:

```
hostname: aperturecomputemodule000
```

On PXEServer, create an override in the shared NFS root:

```
sudo mkdir -p /srv/nfs/default/etc/cloud/cloud.cfg.d

sudo tee /srv/nfs/default/etc/cloud/cloud.cfg.d/99-daspi-hostname.cfg > /dev/null <<'EOF'
#cloud-config
preserve_hostname: true
EOF
```

This instructs cloud-init not to modify the hostname assigned through PXE.

Because all four Raspberry Pis share the same NFS root, this one change applies to every module.

## 4. Remove the shared static hostname

On PXEServer, back up and remove the existing hostname file:

```
if [ -f /srv/nfs/default/etc/hostname ]; then
    sudo mv /srv/nfs/default/etc/hostname \
        /srv/nfs/default/etc/hostname.bak
fi
```

The shared `/etc/hostname` should remain absent so it cannot override the individual PXE-assigned hostnames.

Do not use `hostnamectl set-hostname` to configure individual nodes permanently while they share this writable filesystem.

## 5. Reboot and verify each Raspberry Pi

Reboot each module individually, starting with module 003.

```
ssh daspi@10.0.2.7 'sudo reboot'
```

After reconnecting:

```
hostname
hostnamectl status
cat /proc/cmdline | grep -o 'systemd.hostname=[^ ]*'
ls -l /etc/hostname
```

Expected for module 003:

```
aperturecomputemodule003
systemd.hostname=aperturecomputemodule003
```

The shared `/etc/hostname` should not exist.

Repeat for modules 000–002.

## 6. Verify the complete network

From your laptop:

```
for ip in 4 5 6 7; do
    echo "=== 10.0.2.$ip ==="
    ssh daspi@10.0.2.$ip \
        'hostname; grep -o "systemd.hostname=[^ ]*" /proc/cmdline'
done
```

Expected final configuration:

| IP address | Hostname                   |
| ---------- | -------------------------- |
| `10.0.2.4` | `aperturecomputemodule000` |
| `10.0.2.5` | `aperturecomputemodule001` |
| `10.0.2.6` | `aperturecomputemodule002` |
| `10.0.2.7` | `aperturecomputemodule003` |

## Summary of the permanent changes

| File or service                                                | Modification                                  |
| -------------------------------------------------------------- | --------------------------------------------- |
| `/srv/tftp/config.txt`                                         | Select command-line files by Pi serial number |
| `/srv/tftp/cmdline-XXXXXXXX.txt`                               | Supply each Pi's unique `systemd.hostname`    |
| `set-hostname-from-dhcp.service`                               | Disabled and masked                           |
| `/srv/nfs/default/etc/cloud/cloud.cfg.d/99-daspi-hostname.cfg` | Set `preserve_hostname: true`                 |
| `/srv/nfs/default/etc/hostname`                                | Removed after backup                          |

Result: All four ApertureComputeModules now have persistent, unique hostnames while continuing to boot from the same NFS root.

One outstanding issue, separate from hostname management, is assigning unique `/etc/machine-id` values to the four machines.
