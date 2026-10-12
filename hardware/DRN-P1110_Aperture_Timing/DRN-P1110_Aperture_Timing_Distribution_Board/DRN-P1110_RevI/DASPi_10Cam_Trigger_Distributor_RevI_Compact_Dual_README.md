# DASPi 10-camera trigger distributor — Rev I Compact Dual

Rev I corrects the bypass-capacitor routing identified in the Rev H FreeCAD render.

## Correction

The **VCC-side pad of C3** (Circuit A 0.1 µF bypass capacitor) was present physically in Rev H, but its short copper trace stopped before it reached the existing U1 1.8 V rail.

Rev I adds the missing vertical copper segment:

`C3 V18 pad → U1 pin-20/VCC rail`

The same latent issue existed symmetrically on Circuit B, so Rev I also corrects:

`C6 V18 pad → U2 pin-20/VCC rail`

The GND sides of C3 and C6 remain connected to their respective independent ground rails.

## Bypass capacitor connections

- **C3:** `1.8V_A ↔ 0.1 µF ↔ GND_A`
- **C6:** `1.8V_B ↔ 0.1 µF ↔ GND_B`

There is still no electrical connection between Circuit A and Circuit B.

## Rev I validation

- Different-net copper overlaps: **0**
- Minimum different-net copper clearance: **0.70 mm** (`B_GND` vs `CH10_POST`)
- Minimum A↔B copper separation: **7.00 mm**
- Minimum mounting-hole↔copper clearance: **1.20 mm**
- C3 V18 route continuity check: **PASS**
- C6 V18 route continuity check: **PASS**

All other Rev H Compact Dual architecture is retained: 110 × 48 mm board, one 2-inch copper sheet, two MCP1700 regulators, two independent five-camera circuits, six physical capacitor footprints, vertical 68 Ω resistors, and JST PA camera connectors.
