# DASPi 10-camera trigger distributor — Rev G

Rev G changes the ten camera connectors from JST PLI to the more readily available **JST PA series** while retaining the Rev F regulator, two-inch copper-tape overlap system, solder lands, and forge alignment pins.

## Camera connector set

Use:

- **Board header:** `B02B-PASK-1` — 2-position, top-entry, through-hole PA header with locating boss
- **Cable housing:** `PAP-02V-S`
- **Crimp contact:** `SPHD-002T-P0.5` for **AWG 28–24**
- **Recommended cable:** 28 AWG twisted pair
- **Pin 1:** XTR
- **Pin 2:** GND

JST lists the PA family as a 2.0 mm-pitch secure-lock wire-to-board family, with AWG 28–22 support. The `SPHD-002T-P0.5` contact is specified for AWG 28–24. citeturn737575view0turn560182view0

## Exact PA footprint data used

For the two-position top-entry header, JST specifies:

- Electrical pin pitch: **2.00 mm**
- Electrical PCB hole: **Ø0.7 +0.1 mm** → Rev G uses **Ø0.80 mm**
- Two-position header dimension A: **2.0 mm**
- Two-position overall dimension B: **6.0 mm**
- Header body depth: **5.3 mm**
- Header height above PCB: **7.7 mm**
- Applicable PCB thickness: **1.6 mm**
- With-boss PCB relief: JST calls for approximately **Ø1.1 +0.1 mm**. Rev G uses a **2.4 × 7.0 mm through-slot**, positioned 1.7 mm behind the electrical pin row, so it covers the boss location in either header orientation without depending on FDM hole accuracy.

These values come from JST's PA header and PCB-layout drawings. citeturn894343view0turn144748view0

## Printed-board seating pocket

The overall carrier remains **2.4 mm** thick.

Each PA header receives a **0.8 mm deep component-side seating recess**, leaving:

**2.4 − 0.8 = 1.6 mm local board thickness**

at the connector pins, matching JST's specified applicable PCB thickness. citeturn737575view0

The two Ø0.80 mm electrical holes position the header. The molded locating boss receives the oversized 2.4 × 7.0 mm relief slot rather than a tight precision hole.

## Connector arrangement

The ten headers remain in two five-connector banks.

- Connector center spacing: **7.0 mm**
- PA body width along the two-pin row: **6.0 mm**
- Minimum body-to-body gap: **1.0 mm**
- Pin copper pad diameter: **1.30 mm**
- Edge-to-edge copper clearance between XTR and GND connector pads: **0.70 mm**

## Electrical assignment

```text
74LVC245 Bx ── 68 Ω ── PA pin 1 ── camera XTR
GND ─────────────────── PA pin 2 ── camera GND
```

Keep each camera cable as a twisted XTR/GND pair.

## Rev F 2-inch copper tape system retained

- Visible upper-tape seam: **Y=47.0 mm**
- Lower tape hidden edge: **Y=52.6 mm**
- Overlap: **5.6 mm**
- One 6 mm TRIGGER seam land
- Three 6 mm GND seam stitch lands
- Upper sheet installed second
- Do not solder continuously across the seam

## Geometry check

- Different-net copper overlaps: **0**
- Minimum different-net copper clearance: **0.70 mm** (`CH1_POST` vs `GND`)
- PA XTR/GND pin-pad clearance: **0.70 mm**
- Minimum mounting-hole-to-copper clearance: **2.43 mm**

## Other electronics retained

- MCP1700-1802E/TO 1.8 V regulator
- 1 µF CIN
- 1 µF COUT
- 2 × SN74LVC245A
- 0.1 µF bypass capacitor per transceiver
- 10 × 68 Ω series resistors
- one insulated 1.8 V jumper U1 → U2

## Rev G final mechanical note

The FreeCAD reference model now uses the PA header's **7.7 mm body height** and **5.3 mm body depth**. The connector body is reference geometry only; the printable carrier is governed by the electrical holes, 1.6 mm local seating thickness, and the oversized boss relief.
