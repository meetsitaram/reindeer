# Bill of materials

Generated from `spec/robot.py` by `python -I tools/make_docs.py`. **Nothing here is ordered, and no listing has been re-read for this robot:** prices and links are duck-droids' (the same parts), and each needs the user's look before ordering.

| item | part | maker / number | specification | qty | unit | unit price | line | source |
|---|---|---|---|---|---|---|---|---|
| 1 | Smart servo | ROBOTIS XL330-M288-T | 0.52 N m stall, 103 rev/min at 5 V, 18 g; with its horn and 16 screws | 13 | each | $27.49 | $357.37 | [ROBOTIS US](https://robotis.us/products/dynamixel-xl330-m288-t): sold out on 2026-11-03 (duck-droids), Seeed second source |
| 2 | Idler (rear horn) | ROBOTIS FPX330-H101 | 11 rear horns | 3 | set of 4 | $11.27 | $33.81 | [ROBOTIS US](https://robotis.us/fpx330-h101-4pcs-set/) |
| 3 | Computer | Raspberry Pi Zero 2 W | without header, 65 x 30 mm | 1 | each | $15.00 | $15.00 | [PiShop](https://www.pishop.us/product/raspberry-pi-zero-2-w/) |
| 4 | Servo bus board | Waveshare Bus Servo Adapter (A), SKU 25514 | USB-C to the Pi, 2 servo sockets. Its silkscreen says 9-12.6 V: bench test on 2S | 1 | each | $9.05 | $9.05 | [Abra](https://abra-electronics.com/electromechanical/motors/motor-controllers/wave-25514-serial-bus-servo-driver-board.html) |
| 5 | 5 V regulator for the Pi | hiBCTR USB buck converter QC3.0 | 6-32 V in, 35 x 17 x 8.5 mm (where it goes: open item) | 1 | pack of 2 | $7.97 | $7.97 | Amazon (duck-droids' choice) |
| 6 | IMU, for walking | BNO085 module | I2C, 25.6 x 22.7 mm (where it goes: open item) | 1 | each | $29.50 | $29.50 | [Adafruit 4754](https://www.adafruit.com/product/4754) |
| 7 | Servo cables | ROBOTIS Robot Cable-3P JST EH | 3 legs of the chain plus head: lengths to measure | 1 | set | - | - | ROBOTIS |
| 8 | PLA filament | any | fawn, white, red, green, dark green, dark grey: about 0.3 kg | 1 | spools | - | - | - |

**Known total: $452.70**, without the battery, cables and filament.

## Open items

- **Battery:** a 2S LiPo of at most 44 x 30 x 8 mm, with its plug matching the bus board's input. The space is in the back, on the tray. duck-droids' OVONIC 2S 450 mAh is 61.5 mm long, which doesn't fit. Not chosen: compare 2 or 3 real packs at this size before ordering.
- **XL330s on 2S:** 7.4-8.4 V is past ROBOTIS's 6 V maximum. microduck and duck-droids run them so; or add a 5-6 V, 5 A regulator.
- **Where the regulator and the IMU go:** not modelled yet.
- **Screws:** in the servo boxes. Screws for the printed parts are not designed yet (phase 3).

## Printed parts

See `prints/PRINTS.md`.
