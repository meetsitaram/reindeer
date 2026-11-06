# Standing through the moves (MuJoCo)

max tilt 0.75 deg, drift 0.3 mm, servo force at most 79 % of stall, tracking error at most 0.068 rad

limits: tilt < 5 deg, drift < 5 mm, force < 95 % of stall. kp 6.0 N m/rad (an estimate). Moves: {'head_roll': 0.35, 'head_yaw': 0.6}, {'head_roll': -0.35, 'head_yaw': -0.6, 'neck_pitch': 0.27}, {'L_front_hip': 0.5, 'L_front_knee': -1.0, 'R_front_hip': 0.5, 'R_front_knee': -1.0, 'L_rear_hip': 0.5, 'L_rear_knee': -1.0, 'R_rear_hip': 0.5, 'R_rear_knee': -1.0}, {}, {'L_front_hip': -0.6, 'L_front_knee': 1.2, 'neck_pitch': -0.3}

0 failing
