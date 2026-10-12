# DASPi 10-camera trigger distributor — Rev F

Rev F keeps the Rev E JST PLI locking connectors and adds a **purpose-built seam system for 2-inch (50.8 mm) copper tape**.

## Tape placement

The board is 96 mm tall. Two 50.8 mm tape strips give 101.6 mm of total width, so we have **5.6 mm of overlap** available.

Use this placement:

- **Lower tape first:** approximately `Y = 1.8 → 52.6 mm`
- **Upper tape second:** `Y = 47.0 → 97.8 mm`
- Physical overlap: **5.6 mm**
- Visible lower edge of the upper tape: **Y = 47.0 mm**

The upper tape goes on **second**, so its lower edge is the seam you actually solder.

The carrier now has small edge notches at:

- `Y = 47.0 mm` — visible solder seam / upper-tape lower edge
- `Y = 52.6 mm` — hidden end of the lower tape

There is also a visual-only `Tape_Overlap_Zone_REF` object in FreeCAD.

## Why Y = 47 mm

Rev E already has its center GND rail at Y=47 mm. Putting the visible tape seam exactly there gives us a large, safe ground area for soldering the two sheets together.

Only two electrical networks need to cross from one tape strip to the other:

1. **TRIGGER**
2. **GND**

The 1.8 V connection between U1 and U2 remains an insulated jumper, so it does not depend on the tape seam.

## New seam solder lands

### Trigger

A **6 mm diameter trigger solder land** is centered at:

`X = 14 mm, Y = 47 mm`

Solder the upper-tape edge to the exposed lower tape at this land. A small tinned-wire bridge may also be used.

### Ground

Three **6 mm diameter GND stitch lands** are centered on the Y=47 mm ground rail:

- X=87 mm
- X=112 mm
- X=124 mm

You do not need to solder the whole 136 mm seam. I recommend soldering **at least two of the three GND stitch lands**, with all three preferred for a flight article.

## Suggested soldering sequence

1. Install and form the **lower** 2-inch tape sheet.
2. Install the **upper** sheet second, aligning its lower edge to the Y=47 edge marks.
3. Press/form the upper sheet.
4. Remove copper from the high lands as usual.
5. At the trigger seam land, lightly tin both exposed copper surfaces and bridge the tape edge.
6. Tack the GND seam at the three widened GND lands.
7. Meter:
   - trigger above seam ↔ trigger below seam
   - GND above seam ↔ GND below seam
   - trigger ↔ GND must remain open
8. Only then install components.

Avoid soldering continuously across the whole tape seam. The localized lands minimize heat input into the printed carrier.

## Geometry check after adding the seam lands

- Different-net copper overlaps: **0**
- Minimum different-net copper clearance: **0.70 mm** (`CH1_POST` vs `GND`)
- Trigger seam-land clearance to any other net: **13.59 mm**
- Minimum GND seam-land clearance to any other net: **1.50 mm**
- Minimum mounting-hole-to-copper clearance: **2.43 mm**

## Existing Rev E hardware retained

- 10 × JST PLI `B02B-PLISK-1` locking through-hole headers
- matching `PLIP-02V-S` cable housings
- 28 AWG XTR/GND twisted pairs
- MCP1700-1802E/TO 1.8 V regulator
- 1 µF CIN and 1 µF COUT
- 2 × SN74LVC245A
- 0.1 µF bypass per SN74LVC245
- 10 × 68 Ω XTR series resistors
- forge alignment pins
