# DASPi 10-camera trigger distributor — Rev E

Rev E replaces the ten solder-wire camera exits with **ten locking JST PLI 2-pin wire-to-board connectors**.

## Connector choice

The PLI series is a 2.0 mm-pitch, through-hole wire-to-board connector with a **secure lock**, inertia-lock mechanism and low-profile construction. JST specifies a 9.6 mm mated mounting height and AWG 28–22 wire range. citeturn412598view0

For each camera:

- Header: **B02B-PLISK-1**
- Cable housing: **PLIP-02V-S**
- Contact for AWG 28–24: **SPLI-002T-P0.5**
- Pin 1: **XTR**
- Pin 2: **GND**

JST lists B02B-PLISK-1 as the 2-circuit header and PLIP-02V-S as the corresponding 2-circuit socket housing. citeturn412598view0

## Why PLI instead of JST-GH

JST-GH has an excellent secure lock, but the board header is SMT at 1.25 mm pitch. That is poorly matched to a hand-formed copper-tape PCB. JST PLI gives us the lock while remaining through-hole. JST describes PLI as through-hole with secure/inertia locking. citeturn987338search0

## Important printed-board accommodation

JST specifies **1.6 mm applicable PCB thickness** for PLI. The printed carrier remains 2.4 mm overall, but Rev E cuts a **0.8 mm component-side seating pocket underneath every connector**, leaving 1.6 mm local thickness at the header posts. citeturn412598view0

The electrical post holes are modeled at **0.80 mm**. JST's reference PCB layout specifies **0.75 +0.05 mm** for the 2-circuit version. citeturn412598view0

The locating boss uses an **oversized relief slot** rather than a close-fit round hole. The electrical pins set connector position; the relief simply guarantees that the molded boss cannot interfere with the printed carrier.

## Connector spacing

The ten headers are arranged in two banks:

- J1–J5: upper bank
- J6–J10: lower bank

Connector body pitch within each bank: **7.0 mm**

Nominal body-to-body gap:
- upper bank: **1.0 mm**
- lower bank: **1.0 mm**

Each connector is oriented so its two electrical pins are vertical:

```text
       Jn
   ┌────────┐
   │ LOCK   │
   │        │
   └────────┘
      ○  pin 1  XTR
      ○  pin 2  GND
```

## Wire

Use **28 AWG stranded wire** for each XTR/GND twisted pair. This matches the previously selected light signal wiring and falls within the SPLI-002T-P0.5 contact's AWG 28–24 range. JST lists the PLI family for AWG 28–22 overall. citeturn412598view0

## Copper geometry check

- Different-net overlaps: **0**
- Minimum different-net copper clearance: **0.70 mm** (`CH1_POST` vs `GND`)
- PLI pin-pad edge-to-edge clearance: **0.70 mm**
- Minimum mounting-hole-to-copper clearance: **2.43 mm**

The PLI two-pin footprint is now the tightest copper area. Inspect and meter those connector pads carefully after forming/removing the foil.

## Existing Rev D electronics retained

- MCP1700-1802E/TO creates 1.8 V from Pico 3V3
- 1 µF ceramic CIN
- 1 µF ceramic COUT
- two SN74LVC245A
- 0.1 µF bypass capacitor across pins 20/19 of each SN74LVC245
- ten 68 Ω XTR series resistors
- one insulated 1.8 V jumper from U1 to U2
- forge alignment pins
