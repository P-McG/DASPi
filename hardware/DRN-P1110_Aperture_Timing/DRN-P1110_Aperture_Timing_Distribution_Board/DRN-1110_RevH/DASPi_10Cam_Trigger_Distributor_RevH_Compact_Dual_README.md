# DASPi 10-camera trigger distributor — Rev H Compact Dual FINAL

Corrected Rev H now includes **physical capacitor holes, copper lands, routes, and 3D reference solids**.

The six capacitor footprints are:

| Ref | Value | Purpose |
|---|---:|---|
| C1 | 1 µF | Circuit A MCP1700 input |
| C2 | 1 µF | Circuit A MCP1700 output |
| C3 | 0.1 µF | Circuit A SN74LVC245 local bypass |
| C4 | 1 µF | Circuit B MCP1700 input |
| C5 | 1 µF | Circuit B MCP1700 output |
| C6 | 0.1 µF | Circuit B SN74LVC245 local bypass |

Each uses **2.54 mm lead spacing**, **0.90 mm through-holes**, and **1.80 mm copper lands**. The smaller 1.80 mm land was chosen deliberately to preserve hand-formable copper clearance at the 2.54 mm pitch.

Use small **radial through-hole ceramic capacitors**. They are non-polarized.

Rev H remains **110 × 48 mm**, uses one 2-inch copper sheet, and retains two electrically independent 5-camera circuits.

Updated geometry check:

- Different-net copper overlaps: **0**
- Minimum different-net clearance: **0.70 mm** (`B_GND` vs `CH10_POST`)
- Minimum A↔B separation: **7.00 mm**
- Minimum mounting-hole↔copper clearance: **1.20 mm**
