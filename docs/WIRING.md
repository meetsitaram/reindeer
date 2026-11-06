# Wiring

One Dynamixel bus (Protocol 2.0), from the Waveshare Bus Servo Adapter (A) on the Pi's USB. Each XL330 has 2 sockets, so the chain splits only at the board's 2 sockets:

- **Socket 1:** neck_pitch, head_yaw, head_roll, then jaw and antlers. From head_roll, one lead to the jaw and the antlers each needs a branch: the roll servo has only 2 sockets, so the jaw continues from the antlers.
- **Socket 2:** the legs. Front left hip, knee; across the keel to the front right hip, knee. Then rear right, rear left. 8 servos.

**Power:** a 2S pack on the board's green terminals; the buck converter from the same terminals to the Pi's PWR USB.

Joints in the spec, in order: neck_pitch -> head_yaw -> head_roll -> jaw -> antlers -> L_front_hip -> L_front_knee -> L_rear_hip -> L_rear_knee -> R_front_hip -> R_front_knee -> R_rear_hip -> R_rear_knee.

Not checked yet (phase 3): each lead's length and route through the joints, the plugs' room at each socket, and each rail's stall current. 13 servos stalling would draw about 19 A at 5 V.
