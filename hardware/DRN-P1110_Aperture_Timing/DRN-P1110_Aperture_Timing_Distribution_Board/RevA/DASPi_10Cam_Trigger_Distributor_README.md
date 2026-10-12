# DASPi 10-camera trigger distributor — first-pass FreeCAD design

This package contains a FreeCAD macro for a **3D-printed insulating carrier with recessed copper-tape traces**, plus a matching copper-forming press.

## Architecture

- Pico trigger output: **3.3 V**
- 2 × **SN74LVC245A / 74LVC245, PDIP-20**
- 74LVC245 supply: **regulated 1.8 V**
- `DIR` → 1.8 V (A → B)
- `/OE` → GND
- `A1..A5` on both chips → common Pico trigger
- `B1..B5` → 10 independent camera outputs
- **68 Ω series resistor** in every XTR output
- Every camera gets its own **XTR + GND twisted pair**
- `A6..A8` are tied to GND so unused inputs do not float
- 0.1 µF decoupling capacitor at each 74LVC245

### Why there are two wire jumpers

This is intentionally a single-sided copper layout. The **1.8 V supply is distributed with two short insulated wires** from the 1.8 V input pad to U1 and U2. That is preferable to introducing copper-trace crossings.

The 1.8 V input must come from a **real regulated 1.8 V source**. Do not use a resistor divider as the IC supply.

A small 1.8 V LDO such as an MCP1700-1802 can be mounted separately or added to the design later after the signal distributor is proven.

## Main dimensions

| Parameter | Value |
|---|---:|
| Carrier | 118 × 76 × 2.4 mm |
| Copper groove depth | 0.35 mm |
| Main signal groove | 2.4 mm |
| DIP fan-out neck | 1.2 mm |
| Ground bus | 3.2 mm |
| DIP pin pitch | 2.54 mm |
| DIP row spacing | 7.62 mm |
| 68 Ω resistor hole spacing | 12 mm |
| Mounting holes | 3.2 mm |

The 1.2 mm necks are deliberate. At 2.54 mm DIP pitch, 2–3 mm traces would not leave enough isolation between adjacent IC pins. The routes widen after the fan-out.

## Parts

- 2 × 74LVC245 / SN74LVC245A, PDIP-20
- 10 × 68 Ω axial resistor
- 2 × 0.1 µF through-hole ceramic capacitor
- Regulated 1.8 V supply
- Conductive-adhesive copper foil
- 10 twisted XTR/GND cable pairs
- Plain PC, ASA or PETG carrier material — **not PCCF or other carbon-filled filament**

## 74LVC245 pin use

For each chip:

| Pin(s) | Use |
|---|---|
| 1 | DIR → 1.8 V |
| 2–6 | A1–A5 → Pico trigger |
| 7–9 | A6–A8 → GND |
| 10 | GND |
| 11–13 | B8–B6 unused |
| 14–18 | B5–B1 → five 68 Ω camera outputs |
| 19 | /OE → GND |
| 20 | VCC → 1.8 V |

## Fabrication sequence

1. Run the `.FCMacro` in FreeCAD.
2. Export `Carrier_Print` and `Press_Tool` separately.
3. Print the carrier in plain PC, ASA or PETG.
4. Put conductive-adhesive copper foil over the grooved face.
5. Use the printed press to form the foil into the channels.
6. Remove/sand copper from the high lands so only the recessed copper remains.
7. Check **all adjacent traces for isolation with a multimeter**.
8. Open the component holes through the copper.
9. Install and solder the components from the opposite side.
10. Install the two insulated 1.8 V jumpers shown in the FreeCAD model.
11. Before connecting cameras:
    - verify 1.8 V at both IC VCC pins,
    - verify no XTR output exceeds the intended 1.8 V logic level,
    - verify all ten channels pulse together,
    - verify no shorts between channels.
12. Add cameras progressively: **1 → 2 → 5 → 10**.

## First-prototype warning

This is a prototype fabrication method, not a production PCB. The most critical region is the **2.54 mm DIP footprint**, where the copper isolation is tightest. Inspect it under magnification and meter it before applying power.

All principal mechanical dimensions are parameters near the top of the macro.
