For **13 Raspberry Pis**, I would **not direct-solder power wires to the Raspberry Pi boards** unless this is a final production build with a proper strain-relieved harness. The better approach is:

**high-voltage battery / main DC bus → fused power distribution → local 5.1V buck converters → short USB-C power leads or a proper power header/connector per Pi.**

For Raspberry Pi 5 specifically, Raspberry Pi recommends a **5.1V supply**, and the Pi 5 recommended PSU current capacity is **5A**; it can boot from a good 5V/3A supply, but USB peripheral current is restricted unless a 5V/5A-capable supply is detected or configured. ([Raspberry Pi][1])

### USB-C port vs direct solder

**Use the USB-C power port when possible.**
It is mechanically replaceable, serviceable, and follows the intended power path. For a DASPi-style multi-camera rig, I would use **short, locking/secured USB-C pigtails** fed from your own 5.1V regulators. Do not rely on random USB-C phone chargers or multi-port USB-C bricks; many do not provide the optional **5V/5A** profile that Pi 5 wants. Raspberry Pi’s USB PD white paper notes that Pi SBCs only use **5V power modes**, and 5V/5A is optional in USB-PD, not guaranteed by every charger. ([Raspberry Pi Product Information Portal][2])

**Power through the GPIO 5V pins is acceptable for embedded builds**, but only if you treat it as a real power input: proper connector, multiple ground returns, short wires, fusing, and no Dupont jumper leads. Raspberry Pi documents that GPIO output current is limited, and power requirements rise with camera modules, USB devices, HDMI, etc.; camera modules are listed around **250mA** each. ([Raspberry Pi][1])

**Avoid direct soldering to the Pi PCB** unless there is no other choice. It makes repairs harder, can rip pads under vibration, and removes connector/serviceability advantages. If you want soldered wiring, solder to a **small distribution daughterboard/HAT or custom power plate**, not directly to the Pi.

### Power budget for 13 Pi 5s

Worst-case design number:

`13 × 5V × 5A = 325W`

That is a lot of 5V current:

`13 × 5A = 65A at 5V`

You probably will not draw 65A continuously, but you should not design a flying camera rig around “typical” current only. Raspberry Pi lists Pi 5 typical bare-board active current around **800mA**, but the recommended PSU capacity is **5A**, and low-voltage warnings occur below about **4.63V**, which can cause storage corruption or unpredictable behavior. ([Raspberry Pi][1])

For your drone/camera use, I would plan roughly:

| Design level                |   Per Pi | 13 Pi total | 5V power |
| --------------------------- | -------: | ----------: | -------: |
| Bare/light load             | 1.0–1.5A |      13–20A |  65–100W |
| Camera + Ethernet + compute |     2–3A |      26–39A | 130–195W |
| Full margin Pi 5 design     |       5A |         65A |     325W |

### Recommended architecture

For a drone, do **not** run one huge 5V bus around the aircraft. Current is too high and voltage drop becomes painful. Instead:

1. Use the drone battery voltage, likely **6S or 12S**, as the main distribution voltage.
2. Split into several fused branches.
3. Put **local 5.1V buck regulators** close to groups of Pis.
4. Feed each Pi through **USB-C pigtails** or a proper power header.
5. Fuse each Pi or each small group separately.

A good layout would be:

```text
12S battery / main DC bus
        |
   main fuse + switch / contactor
        |
 power distribution board
        |
  +-----+-----+-----+-----+
  |           |           |
5.1V buck   5.1V buck   5.1V buck ...
3 Pis       3 Pis       3 Pis
  |           |           |
individual fuses / USB-C pigtails
```

For 13 Pis, I would use something like **4 or 5 regulator groups**, not one giant 5V regulator. For example:

| Group  | Load                               |
| ------ | ---------------------------------- |
| Buck A | 3 Pis                              |
| Buck B | 3 Pis                              |
| Buck C | 3 Pis                              |
| Buck D | 3 Pis                              |
| Buck E | 1 Pi + network/control electronics |

Size each 3-Pi buck for at least **15A at 5.1V** if using Pi 5 margin, or **10A minimum** if you know the load is controlled and no USB peripherals are attached.

### My recommendation

For your DASPi rig, I would do this:

**Use a high-voltage battery bus, local 5.1V buck regulators, and short secured USB-C power leads into each Pi.**

That gives you the best mix of reliability, serviceability, and vibration tolerance. Use direct GPIO/header power only where packaging forces it, and avoid direct soldering to the Raspberry Pi itself.

Also add:

* **Individual fuses** per Pi, around 3–5A depending on your measured load.
* **TVS diode / surge protection** on the 5V distribution.
* **Bulk capacitance** near each buck output.
* **Short, thick 5V leads** from regulator to Pi.
* **Common ground** between Pis, cameras, switches, and control electronics.
* **Staggered startup** if all 13 boot at once and the regulators trip from inrush.
* Runtime monitoring with `vcgencmd get_throttled`; any undervoltage flag means the wiring/regulator path is not good enough.

For a bench prototype, USB-C is easiest. For the drone, I would still feed the **USB-C ports**, but from your own regulated 5.1V power distribution rather than from 13 wall adapters.

[1]: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html "Raspberry Pi computer hardware - Raspberry Pi Documentation"
[2]: https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf "USB Power Delivery on Raspberry Pi 5"

