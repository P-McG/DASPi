# DASPi 10-camera trigger distributor — Rev K Compact Dual

Rev K corrects the SN74LVC245A footprints for installation on the back/component side of the carrier. Rev J used conventional top-view DIP numbering even though the reference IC bodies were placed beneath the board, which mirrored the physical pins.

## Mandatory IC orientation

- Install U1 and U2 on the back/component side.
- The notch on each IC must face the bottom edge of the board.
- Viewed from the copper side, pin 1 is lower-left and pin 20 is lower-right.
- Viewed from the component side, pin 1 is lower-right.
- The FreeCAD model includes a pin-1 reference marker for each IC.

Installing either IC with its notch toward the top edge will produce an incorrect pinout.

## SN74LVC245A connections

| Pin | Function | Rev K connection |
| ---: | --- | --- |
| 1 | DIR | 1.8 V; selects A-to-B transmission |
| 2–9 | A1–A8 | Common trigger input |
| 10 | GND | Insulated jumper to the upper local ground bus |
| 11–13 | B8–B6 | Unused; drilled holes without copper pads |
| 14–18 | B5–B1 | Five camera outputs through 68 Ω resistors |
| 19 | /OE | Insulated jumper to the lower local ground landing |
| 20 | VCC | Local regulated 1.8 V |

The insulated jumpers pass above the copper layer and must not contact any intervening trace.

## Output mapping

The common trigger drives all eight A inputs, so reversing the selected B-channel order does not change signal behaviour. It permits a planar, single-layer fan-out without crossed traces.

| Connector | IC output | IC pin |
| --- | --- | ---: |
| J1 / J6 | B5 | 14 |
| J2 / J7 | B4 | 15 |
| J3 / J8 | B3 | 16 |
| J4 / J9 | B2 | 17 |
| J5 / J10 | B1 | 18 |

## Capacitors

- C1/C4: 1 µF from 3.3 V to local GND at the regulator input.
- C2/C5: 1 µF from 1.8 V to local GND at the regulator output.
- C3/C6: 0.1 µF from 1.8 V to local GND beside the corrected physical pin-20 end of U1/U2.

## Retained connector correction

- Ten JST B02B-PASK-1 connectors, each using two 0.80 mm lead holes on 2.00 mm centres.
- No full-depth rectangular connector openings.
- Each connector retains a shallow 0.80 mm underside seating pocket.
- The pocket leaves 1.60 mm local carrier thickness and 1.25 mm of solid material beneath the 0.35 mm copper grooves.

## Main dimensions and construction

- Carrier: 110 × 48 × 2.4 mm.
- Two electrically independent five-camera circuits.
- Two MCP1700-1802E/TO regulators.
- Two back-mounted SN74LVC245AN 20-pin PDIP buffers.
- Ten vertical 68 Ω, 1/4 W resistors.
- Six through-hole capacitor footprints.
- One 2-inch copper sheet; no seam.

## Validation summary

- Python syntax: PASS.
- Corrected back-side pin map: PASS.
- Circuit A/B mirror symmetry: PASS.
- Different-net copper overlaps: 0.
- Minimum different-net copper clearance: 0.70 mm.
- Minimum A-to-B copper separation: 7.00 mm.
- Minimum mounting-hole-to-copper clearance: 1.20 mm.
- Connector lead holes: 20 total, exactly two per connector.

## Use in FreeCAD 1.1

1. Open FreeCAD.
2. Select **Macro → Macros… → Create**, or copy the `.FCMacro` into the FreeCAD macro directory.
3. Run `DASPi_10Cam_Trigger_Distributor_RevK_Compact_Dual.FCMacro`.
4. Inspect `Carrier_Print`, `Press_Tool`, the IC reference bodies, pin-1 markers, and all four jumper references.
5. Save the generated document as an `.FCStd` file.

Before applying power, use continuity mode to verify pins 1, 10, 19, and 20 on both ICs against the table above.
