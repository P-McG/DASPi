# DASPi 10-camera trigger distributor — Rev H Compact Dual FINAL

Rev H is the compact, redundant replacement for Rev G.

## Size reduction

| Revision | Board size | Area |
|---|---:|---:|
| Rev G | 136 × 96 mm | 13056 mm² |
| Rev H | **110 × 48 mm** | **5280 mm²** |

That is a **59.6% reduction in board area**.

The 48 mm height also means **one 2-inch / 50.8 mm copper-tape sheet covers the entire board**, so Rev H eliminates the copper-sheet seam completely.

## Two independent circuits

Circuit A handles cameras 1–5:

`3V3_A / GND_A / TRIG_A → MCP1700-A → U1 → J1–J5`

Circuit B handles cameras 6–10:

`3V3_B / GND_B / TRIG_B → MCP1700-B → U2 → J6–J10`

There is no copper connection between the A and B 3.3 V, 1.8 V, GND or trigger networks.

If both sides are driven from one Pico during development, connect their grounds externally at the Pico. Rev H itself does not join the two sides.

## Two MCP1700 regulators

Each channel gets its own **MCP1700-1802E/TO**:

- pin 1 = GND
- pin 2 = 3.3 V input
- pin 3 = regulated 1.8 V
- CIN = 1 µF ceramic
- COUT = 1 µF ceramic

There is **no U1→U2 1.8 V jumper** anymore.

## Compact 68 Ω resistor mounting

To get the board down to 110 mm wide while keeping the existing **68 Ω, ¼ W axial resistors**, Rev H mounts them **vertically**.

Each resistor uses a **3.81 mm lead pitch**:

```text
component side

      68 Ω body
         │
         │
         │
         ●──── bent lead
         │       │
─────────○───────○──────── board
        R1      R2
       3.81 mm
```

One lead passes straight through the first hole. Bend the other lead down into the second hole.

For an aircraft installation, after the board has passed electrical testing, add a small dab of **neutral-cure silicone or suitable electronics-safe adhesive** against each standing resistor body to reduce vibration loading.

## SN74LVC245A details

Each circuit retains its own SN74LVC245A.

- pin 20 = its local 1.8 V
- pin 10 = its local GND
- pin 1 DIR = its local 1.8 V
- pin 19 /OE = its local GND

For space, pin 19 uses a **short insulated component-side jumper** to a nearby ground landing pad.

Pins 11–13 are the unused B6–B8 outputs. Rev H keeps the physical DIP holes but intentionally removes copper lands from those three pins to create more routing clearance.

## JST PA camera connectors

- Board header: `B02B-PASK-1`
- Housing: `PAP-02V-S`
- Contact: `SPHD-002T-P0.5`
- Pin 1 = XTR
- Pin 2 = GND
- 28 AWG twisted XTR/GND pair

J1–J5 belong only to Circuit A.  
J6–J10 belong only to Circuit B.

## Final geometry validation

- Different-net copper overlaps: **0**
- Minimum different-net copper clearance: **0.70 mm** (`B_GND` vs `CH10_POST`)
- Minimum A↔B copper separation: **7.00 mm**
- PA XTR↔GND pad clearance: **0.70 mm**
- Minimum unused-DIP-hole↔copper clearance: **1.17 mm**
- Minimum mounting-hole↔copper clearance: **1.20 mm**

## First power-up

1. Leave U1 and U2 out.
2. Power Circuit A only; verify ~1.8 V on MCP1700-A pin 3.
3. Power Circuit B only; verify ~1.8 V on MCP1700-B pin 3.
4. Confirm no continuity A 1.8 V ↔ B 1.8 V.
5. Confirm no continuity A 3V3 ↔ B 3V3.
6. If using genuinely isolated upstream supplies, confirm no continuity GND_A ↔ GND_B.
7. Install U1/U2 and the local capacitors.
8. Install the two /OE ground jumpers.
9. Test J1–J5 independently.
10. Test J6–J10 independently.
