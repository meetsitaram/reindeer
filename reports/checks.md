# Physical checks

spec: `spec/robot.json`, 2026-11-05 23:16, 13 s

108 checks: 106 pass, 2 warn, **0 fail**

## targets

| status | item | detail |
|---|---|---|
| PASS | height | 239 mm (aim 240 mm +/- 5 %) |
| PASS | crown_z | 0.759 H (reference 0.780 H +/- 12 %; walking 5.0 s: the fur's top) |
| PASS | ear_span | 0.600 H (reference 0.540 H +/- 12 %; walking 0.0 s and sitting 0.0 s: ear tip to ear tip) |
| PASS | head_width | 0.442 H (reference 0.440 H +/- 12 %; walking 0.0 s: cheeks (sitting 0.0 s: 0.42)) |
| PASS | eye_z | 0.560 H (reference 0.550 H +/- 12 %; walking 5.0 s side 0.51, 0.0 s front 0.63 (low camera)) |
| PASS | eye_spacing | 0.190 H (reference 0.190 H +/- 12 %; walking 0.0 s: centre to centre (sitting 0.18)) |
| PASS | eye_d | 0.127 H (reference 0.130 H +/- 12 %; sitting 0.0 s: the green ring with its white rim, 0.12; walking 0.0 s 0.14) |
| PASS | chin_z | 0.389 H (reference 0.360 H +/- 12 %; walking 5.0 s: the muzzle's lowest point) |
| PASS | back_z | 0.451 H (reference 0.450 H +/- 12 %; walking 5.0 s: the back's top behind the head) |
| WARN | belly_z | 0.238 H (reference 0.190 H +/- 12 %; walking 5.0 s: the belly between the legs). DEVIATION: two XL330s a leg (thigh 42 mm, shin 34 mm + hoof) put the hips at 70 mm and the belly at 57 mm, 0.24 H: longer legs than the plush, kept |
| PASS | body_length | 0.551 H (reference 0.500 H +/- 12 %; walking 5.0 s: rump to chest, head and tail left out) |

## mass

| status | item | detail |
|---|---|---|
| PASS | total | 625 g bottom-up (budget 650 g) |

## printable

| status | item | detail |
|---|---|---|
| PASS | largest part | 87 printed parts; largest body: 132 mm, on a 256 mm bed |

## axis

| status | item | detail |
|---|---|---|
| PASS | neck_pitch | XL330_M288 0.0 mm off the axis |
| PASS | head_yaw | XL330_M288 0.0 mm off the axis |
| PASS | head_roll | XL330_M288 0.0 mm off the axis |
| PASS | L_front_hip | XL330_M288 0.0 mm off the axis |
| PASS | L_front_knee | XL330_M288 0.0 mm off the axis |
| PASS | L_rear_hip | XL330_M288 0.0 mm off the axis |
| PASS | L_rear_knee | XL330_M288 0.0 mm off the axis |
| PASS | R_front_hip | XL330_M288 0.0 mm off the axis |
| PASS | R_front_knee | XL330_M288 0.0 mm off the axis |
| PASS | R_rear_hip | XL330_M288 0.0 mm off the axis |
| PASS | R_rear_knee | XL330_M288 0.0 mm off the axis |

## static

| status | item | detail |
|---|---|---|
| PASS | zero pose | 0 overlaps |

## covers

| status | item | detail |
|---|---|---|
| PASS | motors clear of the covers | 0 cover / motor crossings |

## connect

| status | item | detail |
|---|---|---|
| PASS | body structure |  |
| PASS | body covers mounted |  |
| PASS | neck structure |  |
| PASS | neck covers mounted |  |
| PASS | yawlink structure |  |
| PASS | yawlink covers mounted |  |
| PASS | head structure |  |
| PASS | head covers mounted |  |
| PASS | L_front_thigh structure |  |
| PASS | L_front_thigh covers mounted |  |
| PASS | L_front_shin structure |  |
| PASS | L_front_shin covers mounted |  |
| PASS | L_rear_thigh structure |  |
| PASS | L_rear_thigh covers mounted |  |
| PASS | L_rear_shin structure |  |
| PASS | L_rear_shin covers mounted |  |
| PASS | R_front_thigh structure |  |
| PASS | R_front_thigh covers mounted |  |
| PASS | R_front_shin structure |  |
| PASS | R_front_shin covers mounted |  |
| PASS | R_rear_thigh structure |  |
| PASS | R_rear_thigh covers mounted |  |
| PASS | R_rear_shin structure |  |
| PASS | R_rear_shin covers mounted |  |
| PASS | neck_pitch axis |  |
| PASS | head_yaw axis |  |
| PASS | head_roll axis |  |
| PASS | L_front_hip axis |  |
| PASS | L_front_knee axis |  |
| PASS | L_rear_hip axis |  |
| PASS | L_rear_knee axis |  |
| PASS | R_front_hip axis |  |
| PASS | R_front_knee axis |  |
| PASS | R_rear_hip axis |  |
| PASS | R_rear_knee axis |  |

## skin

| status | item | detail |
|---|---|---|
| PASS | battery 1S 1000 mAh (buy) | 0.00 % outside the skin |
| PASS | bus board (buy) | 0.00 % outside the skin |
| PASS | head_roll actuator (buy) XL330_M288 | 0.00 % outside the skin |
| PASS | head_roll rear horn (buy) FPX330-H101 | 0.00 % outside the skin |
| PASS | head_roll flange | 0.00 % outside the skin |
| PASS | head bracket | 0.00 % outside the skin |
| PASS | head crown post | 0.00 % outside the skin |
| PASS | head face post | 0.02 % outside the skin |

## sweep

| status | item | detail |
|---|---|---|
| PASS | neck_pitch | [-0.30, +0.27] rad clear |
| PASS | head_yaw | [-1.00, +1.00] rad clear |
| PASS | head_roll | [-0.35, +0.35] rad clear |
| PASS | L_front_hip | [-0.60, +0.60] rad clear |
| PASS | L_front_knee | [-1.30, +1.30] rad clear |
| PASS | L_rear_hip | [-0.60, +0.60] rad clear |
| PASS | L_rear_knee | [-1.30, +1.30] rad clear |

## poses

| status | item | detail |
|---|---|---|
| PASS | stand: balance | CoM 32 mm inside what touches the floor |
| PASS | stand: within limits | yes |
| PASS | stand: collisions | none |
| PASS | crouch: balance | CoM 23 mm inside what touches the floor |
| PASS | crouch: within limits | yes |
| PASS | crouch: collisions | none |
| PASS | nod down: balance | CoM 32 mm inside what touches the floor |
| PASS | nod down: within limits | yes |
| PASS | nod down: collisions | none |
| PASS | look up: balance | CoM 32 mm inside what touches the floor |
| PASS | look up: within limits | yes |
| PASS | look up: collisions | none |
| PASS | head tilt, looking round: balance | CoM 28 mm inside what touches the floor |
| PASS | head tilt, looking round: within limits | yes |
| PASS | head tilt, looking round: collisions | none |
| PASS | paw lifted (three legs): balance | CoM 14 mm inside what touches the floor |
| PASS | paw lifted (three legs): within limits | yes |
| PASS | paw lifted (three legs): collisions | none |
| WARN | trot, diagonal pair down: balance | two hooves down: dynamic balance, not static |
| PASS | trot, diagonal pair down: within limits | yes |
| PASS | trot, diagonal pair down: collisions | none |

## torque

| status | item | detail |
|---|---|---|
| PASS | neck_pitch | 0.07 N m in 'nod down' vs rated 0.17 N m (1 x XL330_M288): 42 % used |
| PASS | R_front_hip | 0.02 N m in 'crouch' vs rated 0.17 N m (1 x XL330_M288): 12 % used |
| PASS | L_front_hip | 0.02 N m in 'crouch' vs rated 0.17 N m (1 x XL330_M288): 12 % used |
| PASS | L_rear_hip | 0.02 N m in 'crouch' vs rated 0.17 N m (1 x XL330_M288): 12 % used |
| PASS | R_rear_hip | 0.02 N m in 'crouch' vs rated 0.17 N m (1 x XL330_M288): 12 % used |
| PASS | head_roll | 0.01 N m in 'head tilt, looking round' vs rated 0.17 N m (1 x XL330_M288): 4 % used |
| PASS | R_rear_knee | 0.01 N m x 1.5 in 'trot, diagonal pair down' vs peak 0.52 N m (1 x XL330_M288): 2 % used |
| PASS | L_front_knee | 0.01 N m x 1.5 in 'trot, diagonal pair down' vs peak 0.52 N m (1 x XL330_M288): 2 % used |
| PASS | L_rear_knee | 0.00 N m in 'crouch' vs rated 0.17 N m (1 x XL330_M288): 1 % used |
| PASS | R_front_knee | 0.00 N m in 'crouch' vs rated 0.17 N m (1 x XL330_M288): 1 % used |
| PASS | head_yaw | 0.00 N m in 'look up' vs rated 0.17 N m (1 x XL330_M288): 0 % used |
