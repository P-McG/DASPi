images/
├── base/
│   ├── raspios-lite-64.img
│   └── notes.md
│
├── golden/
│   ├── daspi-v1.0.img
│   ├── daspi-v1.1.img
│   └── latest.img -> daspi-v1.1.img
│
├── extracted/
│   ├── daspi-v1.1/
│   │   ├── bootfs/
│   │   └── rootfs/
│
├── overlays/
│   ├── firstboot/
│   │   ├── firstboot.sh
│   │   └── firstboot.service
│   │
│   ├── networking/
│   │   └── dhcpcd.conf
│   │
│   └── services/
│       └── daspi.service
│
├── builds/
│   ├── build.sh
│   └── config.env
│
└── pxe/
    ├── tftp/
    └── nfs/
