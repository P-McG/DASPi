# DASPi 10-camera trigger distributor — Rev J Compact Dual

Rev J removes the unnecessary full-depth rectangular openings beneath all ten camera connectors. The purchased connector is the JST `B02B-PASK-1`, a vertical two-position, 2.00 mm-pitch through-hole header.

## Connector-footprint correction

- Removed all ten 7.0 × 2.4 mm full-depth rectangular boss slots.
- Each connector now uses only two 0.80 mm lead holes on 2.00 mm centres.
- Retained a 0.80 mm-deep underside seating pocket beneath each connector, leaving 1.60 mm local carrier thickness as specified for the connector family.
- The seating pocket is not a through-opening; 1.25 mm of solid carrier remains beneath the 0.35 mm top-side copper grooves.
- Rev I copper traces, pads, resistor positions and ground buses are unchanged.

The original full-depth rectangular slots were the source of the trace/opening interference shown in the annotated Rev I image. With those unused features removed, there is no through-opening for a trace to cross.

## Retained Rev I features

- Board size: 110 × 48 × 2.4 mm
- Two electrically independent five-camera circuits
- C3 and C6 V18 pads connected to their corresponding 1.8 V rails
- Two MCP1700-1802E/TO regulators
- Two SN74LVC245A buffers
- Ten JST B02B-PASK-1 connectors
- Six physical capacitor footprints
- Vertical 68 Ω, 1/4 W resistors
- One 2-inch copper sheet; no seam

## Validation

- Macro Python syntax: **PASS**
- Headless macro geometry construction: **PASS**
- Rev I copper segment geometry retained exactly: **PASS**
- Rev I copper pad geometry retained exactly: **PASS**
- Rev I round-hole geometry retained exactly: **PASS**
- Connector footprint check — exactly two 0.80 mm holes per connector: **PASS**
- Full-depth rectangular connector openings remaining: **0**
- Local thickness under connector seating pockets: **1.60 mm**
- Solid web beneath top-side trace grooves: **1.25 mm**
- Copper segments: **126**
- Copper pads: **100**
- Drilled round holes: **108**
- Different-net copper overlaps: **0**
- Minimum different-net copper clearance: **0.70 mm**
- Minimum A-to-B copper separation: **7.00 mm**
- Minimum mounting-hole-to-copper clearance: **1.20 mm**

## Use in FreeCAD 1.1

1. Open FreeCAD.
2. Select **Macro → Macros… → Create** or copy the `.FCMacro` file into your FreeCAD macro directory.
3. Run `DASPi_10Cam_Trigger_Distributor_RevJ_Compact_Dual.FCMacro`.
4. Inspect `Carrier_Print`, `Press_Tool`, and the connector reference objects.
5. Save the generated document as an `.FCStd` file.

The shallow underside pockets reduce only the connector mounting regions from 2.4 mm to 1.6 mm. The remainder of the carrier retains its full 2.4 mm thickness.
