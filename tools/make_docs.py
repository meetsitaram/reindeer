"""BOM.md and docs/WIRING.md from the spec: the bought parts it holds, counted, with what each joint needs.

  python -I tools/make_docs.py

Prices and listings from duck-droids' all-Dynamixel BOM (docs/dynamixel/BOM.md there, read 2026-11-03..05): the same
servos, idlers, computer, bus board and regulator. Nothing here has been re-read at a shop for this robot: every line is
"confirm before ordering", and the user confirms each link (phase 4's gate). Screws: each XL330 comes with 6 M2 x 6 tap
screws for the horns and 10 M2 x 8 tap screws for the frame (ROBOTIS's box contents, as duck-droids records).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402

CATALOG = {
    'XL330_M288': ('Smart servo', 'ROBOTIS XL330-M288-T', '0.52 N m stall, 103 rev/min at 5 V, 18 g; with its horn and 16 screws', 27.49,
                   '[ROBOTIS US](https://robotis.us/products/dynamixel-xl330-m288-t): sold out on 2026-11-03 (duck-droids), Seeed second source'),
    'FPX330-H101': ('Idler (rear horn)', 'ROBOTIS FPX330-H101', 'set of 4', 11.27, '[ROBOTIS US](https://robotis.us/fpx330-h101-4pcs-set/)'),
    'Pi Zero 2 W': ('Computer', 'Raspberry Pi Zero 2 W', 'without header, 65 x 30 mm', 15.00, '[PiShop](https://www.pishop.us/product/raspberry-pi-zero-2-w/)'),
    'bus board': ('Servo bus board', 'Waveshare Bus Servo Adapter (A), SKU 25514', 'USB-C to the Pi, 2 servo sockets. Its silkscreen says 9-12.6 V: bench test on 2S', 9.05,
                  '[Abra](https://abra-electronics.com/electromechanical/motors/motor-controllers/wave-25514-serial-bus-servo-driver-board.html)'),
}


def main():
    spec = G.load()
    joints = spec['joints']
    n_servo = len(joints)
    rear = sum(1 for l in spec['links'].values() for g in l['geoms'] if 'FPX330-H101' in g['name'])
    sets = -(-rear // 4)
    rows, total = [], 0.0

    def row(i, item, part, specs, qty, unit, price, src):
        nonlocal total
        line = qty * price if price else None
        total += line or 0
        rows.append(f"| {i} | {item} | {part} | {specs} | {qty} | {unit} | {'$%.2f' % price if price else '-'} | {'$%.2f' % line if line else '-'} | {src} |")

    c = CATALOG['XL330_M288']
    row(1, c[0], c[1], c[2], n_servo, 'each', c[3], c[4])
    c = CATALOG['FPX330-H101']
    row(2, c[0], c[1], f'{rear} rear horns', sets, c[2], c[3], c[4])
    c = CATALOG['Pi Zero 2 W']
    row(3, c[0], c[1], c[2], 1, 'each', c[3], c[4])
    c = CATALOG['bus board']
    row(4, c[0], c[1], c[2], 1, 'each', c[3], c[4])
    row(5, '5 V regulator for the Pi', 'hiBCTR USB buck converter QC3.0', '6-32 V in, 35 x 17 x 8.5 mm (where it goes: open item)', 1, 'pack of 2', 7.97, 'Amazon (duck-droids\' choice)')
    row(6, 'IMU, for walking', 'BNO085 module', 'I2C, 25.6 x 22.7 mm (where it goes: open item)', 1, 'each', 29.50, '[Adafruit 4754](https://www.adafruit.com/product/4754)')
    row(7, 'Servo cables', 'ROBOTIS Robot Cable-3P JST EH', '3 legs of the chain plus head: lengths to measure', 1, 'set', None, 'ROBOTIS')
    row(8, 'PLA filament', 'any', 'fawn, white, red, green, dark green, dark grey: about 0.3 kg', 1, 'spools', None, '-')
    lines = ['# Bill of materials', '', 'Generated from `spec/robot.py` by `python -I tools/make_docs.py`. **Nothing here is ordered, and no '
             'listing has been re-read for this robot:** prices and links are duck-droids\' (the same parts), and each needs the '
             'user\'s look before ordering.', '',
             '| item | part | maker / number | specification | qty | unit | unit price | line | source |',
             '|---|---|---|---|---|---|---|---|---|'] + rows + [
             '', f'**Known total: ${total:.2f}**, without the battery, cables and filament.', '',
             '## Open items', '',
             '- **Battery:** a 2S LiPo of at most 44 x 30 x 8 mm, with its plug matching the bus board\'s input. The space is in the '
             'back, on the tray. duck-droids\' OVONIC 2S 450 mAh is 61.5 mm long, which doesn\'t fit. Not chosen: compare 2 or 3 real '
             'packs at this size before ordering.',
             '- **XL330s on 2S:** 7.4-8.4 V is past ROBOTIS\'s 6 V maximum. microduck and duck-droids run them so; or add a 5-6 V, 5 A '
             'regulator.',
             '- **Where the regulator and the IMU go:** not modelled yet.',
             '- **Screws:** in the servo boxes. Screws for the printed parts are not designed yet (phase 3).',
             '', '## Printed parts', '', 'See `prints/PRINTS.md`.']
    open(os.path.join(G.ROOT, 'BOM.md'), 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    chain = ' -> '.join(j['name'] for j in joints)
    wiring = ['# Wiring', '', 'One Dynamixel bus (Protocol 2.0), from the Waveshare Bus Servo Adapter (A) on the Pi\'s USB. Each XL330 '
              'has 2 sockets, so the chain splits only at the board\'s 2 sockets:', '',
              '- **Socket 1:** neck_pitch, head_yaw, head_roll, then jaw and antlers. From head_roll, one lead to the jaw and the '
              'antlers each needs a branch: the roll servo has only 2 sockets, so the jaw continues from the antlers.',
              '- **Socket 2:** the legs. Front left hip, knee; across the keel to the front right hip, knee. Then rear right, rear '
              'left. 8 servos.', '',
              '**Power:** a 2S pack on the board\'s green terminals; the buck converter from the same terminals to the Pi\'s PWR USB.',
              '', f'Joints in the spec, in order: {chain}.', '',
              'Not checked yet (phase 3): each lead\'s length and route through the joints, the plugs\' room at each socket, and '
              'each rail\'s stall current. 13 servos stalling would draw about 19 A at 5 V.']
    os.makedirs(os.path.join(G.ROOT, 'docs'), exist_ok=True)
    open(os.path.join(G.ROOT, 'docs', 'WIRING.md'), 'w', encoding='utf-8').write('\n'.join(wiring) + '\n')
    print(f'BOM: {n_servo} servos, {rear} idlers ({sets} sets), known ${total:.2f}; docs/WIRING.md; 0 failing')


if __name__ == '__main__':
    main()
