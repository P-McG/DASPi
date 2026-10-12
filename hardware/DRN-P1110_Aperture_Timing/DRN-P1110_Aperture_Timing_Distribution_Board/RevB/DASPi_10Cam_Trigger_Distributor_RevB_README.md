# DASPi 10-camera trigger distributor — Rev B FINAL

This revision fixes the conflicts visible in Rev A and the initial automated Rev B check.

## Electrical layout

- 2 × SN74LVC245A / 74LVC245 PDIP-20
- VCC = regulated **1.8 V**
- Pico trigger = **3.3 V**
- DIR pin 1 → 1.8 V
- /OE pin 19 → GND
- A1–A8 on both chips receive the common Pico trigger
- B1–B5 provide the ten camera channels
- B6–B8 are unused
- Each XTR output has a **68 Ω** axial series resistor
- Each camera has an adjacent dedicated GND pad
- Each 74LVC245 gets a **0.1 µF ceramic capacitor soldered directly across pins 20 and 19**

Pin 19 is tied to GND, so using pins 20 and 19 gives a very short VCC-to-GND decoupling path without adding large capacitor pads that crowd the DIP footprint.

## Rev B changes

- 1.8 V input moved to the upper-left, completely separate from GND copper.
- Camera output groups separated by a dedicated center ground corridor.
- U1 center ground rail is at Y=46.0 mm.
- CH5 is at Y=52.0 mm and CH6 at Y=40.0 mm.
- Camera XTR/GND pad spacing increased to 7.0 mm center-to-center.
- Ground bus moved to X=121.0 mm.
- Mounting holes moved away from all copper.
- IC body references are now on the component side, opposite the copper face.
- The 1.8 V rail still uses two insulated jumper wires, avoiding single-layer crossings.

## Board dimensions

| Item | Rev B |
|---|---:|
| Board | 130 × 88 × 2.4 mm |
| Copper groove depth | 0.35 mm |
| Main signal groove | 2.4 mm |
| DIP neck groove | 1.0 mm |
| Ground groove | 3.2 mm |
| DIP pad diameter | 1.70 mm |
| General pad diameter | 4.6 mm |
| Mounting hole | 3.2 mm |

## Automated geometry check

- Different-net copper overlaps: **0**
- Absolute minimum different-net copper clearance: **0.84 mm**
- The minimum is at the 2.54 mm DIP-pin fanout, as expected.
- Minimum mounting-hole-to-copper clearance: **2.81 mm**
- Center GND to CH5 large-pad envelope: **2.10 mm**
- Center GND to CH6 large-pad envelope: **2.10 mm**
- Top GND to CH1 large-pad envelope: **2.10 mm**
- Bottom GND to CH10 large-pad envelope: **4.10 mm**
- XTR-to-GND output-pad edge clearance: **2.40 mm**

## Assembly

1. Print `Carrier_Print` using plain PC, ASA or PETG.
2. Print `Press_Tool`.
3. Form conductive-adhesive copper foil into the recessed traces.
4. Remove copper from the high lands.
5. Meter every adjacent net for isolation.
6. Install both 74LVC245 ICs from the component side.
7. Solder 0.1 µF directly across pins 20 and 19 on each IC.
8. Install the ten 68 Ω resistors.
9. Add the two insulated 1.8 V jumpers.
10. Verify 1.8 V at both pin-20 pads.
11. Verify all ten outputs with no cameras connected.
12. Bring cameras online 1 → 2 → 5 → 10.

Use a **regulated 1.8 V source** for `V18_IN`; do not use a resistor divider as the IC supply.
