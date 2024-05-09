from utils.simple_simulator import Quadrotor3D
from utils.trajectories import loop_trajectory
import numpy as np
from config.config import LoopTrajConfig
simple_simulator = Quadrotor3D()
ref_states, ref_ts, ref_us = loop_trajectory(
    quad = simple_simulator,
    discretization_dt = LoopTrajConfig.dt,
    radius = LoopTrajConfig.radius,
    z = LoopTrajConfig.z,
    lin_acc = LoopTrajConfig.lin_acc,
    clockwise = LoopTrajConfig.clockwise,
    yawing = LoopTrajConfig.yawing,
    v_max = LoopTrajConfig.v_max
)



# print(ref_states)

# print(ref_states[0])
# print(ref_states[-1])

# try:
#     while True:
#         print(1)
#         pass
# except KeyboardInterrupt:
#     print("Ctrl+C 被按下，程序退出")

# np.show_config()