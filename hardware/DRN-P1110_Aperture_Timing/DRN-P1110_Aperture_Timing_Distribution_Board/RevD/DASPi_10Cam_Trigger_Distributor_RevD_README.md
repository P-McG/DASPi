# DASPi 10-camera trigger distributor — Rev D

Rev D integrates the **1.8 V regulator directly onto the printed copper board**.

## Pico connection

Only three wires now need to come from the Pico:

| Board input | Pico |
|---|---|
| `GND` | GND |
| `3V3` | `3V3(OUT)` |
| `TRIG` | chosen trigger GPIO/PIO output |

## 1.8 V regulator

Use **MCP1700-1802E/TO**.

TO-92 pinout:

| Pin | Function |
|---|---|
| 1 | GND |
| 2 | VIN |
| 3 | VOUT |

The board routes:

`Pico 3V3 -> MCP1700 VIN -> regulated 1.8 V -> U1 VCC`

U1 then feeds U2 through **one insulated 1.8 V jumper** from U1 pin 20 to U2 pin 20.

### MCP1700 capacitors

Install on the **component side**, close to the regulator:

- `CIN`: **1 µF ceramic**, MCP1700 pin 2 to pin 1
- `COUT`: **1 µF ceramic**, MCP1700 pin 3 to pin 1

A 1 µF ceramic output capacitor is the value specified by Microchip for stable operation. A higher value such as 2.2 µF is also reasonable if desired, but this Rev D documentation uses the datasheet's 1 µF baseline.

## SN74LVC245 power

For each SN74LVC245AN:

- pin 20 = 1.8 V VCC
- pin 1 = DIR, tied to 1.8 V
- pin 19 = /OE, tied to GND
- pin 10 = GND
- 0.1 µF ceramic directly across pins 20 and 19

The Pico's 3.3 V trigger goes to the A inputs; the IC operates from 1.8 V, so the B outputs are 1.8 V logic.

## Rev D geometry

- Board: **130 × 88 × 2.4 mm**
- Copper recess: **0.35 mm**
- Main traces: **2.4 mm**
- DIP/regulator necks: **1.0 mm**
- Ground bus: **3.2 mm**
- Forge alignment pins retained: **2.70 mm shaft into 3.20 mm carrier holes**

## Automated copper check

- Different-net copper overlaps: **0**
- Minimum different-net copper clearance: **0.84 mm** (`CH3_PRE` vs `CH4_PRE`)
- Minimum mounting-hole-to-copper clearance: **2.19 mm**
- Center GND ↔ CH5 clearance: **2.10 mm**
- Center GND ↔ CH6 clearance: **2.10 mm**
- Camera XTR ↔ GND pad clearance: **2.40 mm**
- MCP1700 pin 1 ↔ pin 2 copper-pad clearance: **0.84 mm**
- MCP1700 pin 2 ↔ pin 3 copper-pad clearance: **0.84 mm**

## Assembly order

1. Print `Carrier_Print`.
2. Print `Press_Tool`.
3. Form and isolate the copper foil.
4. Check every adjacent copper net with a meter.
5. Install MCP1700-1802E/TO.
6. Install 1 µF `CIN` and 1 µF `COUT` close to the MCP1700.
7. Power the board from Pico 3.3 V **without the 74LVC245s installed** and confirm approximately 1.8 V at MCP1700 pin 3.
8. Install U1 and U2.
9. Install 0.1 µF decoupling capacitors across pins 20 and 19 of each transceiver.
10. Install the single insulated 1.8 V jumper from U1 pin 20 to U2 pin 20.
11. Confirm 1.8 V at both U1 and U2 pin 20.
12. Verify all ten XTR outputs before connecting cameras.
13. Connect cameras progressively: 1 → 2 → 5 → 10.

## Components added in Rev D

- 1 × MCP1700-1802E/TO
- 2 × 1 µF ceramic capacitor
- No external 1.8 V supply connection is needed.
